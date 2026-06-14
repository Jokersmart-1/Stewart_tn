#include "main.h"
#include "usb_otg.h"
#include "gpio.h"

/* Externs from main.c */
extern AxisConfig_t axes[NUM_AXES];
extern volatile uint8_t simulation_running;

/* Globals for streaming sub-segmenting ping-pong engine */
MoveCommand_t cmd_queue[CMD_QUEUE_SIZE];
volatile uint8_t queue_head = 0;
volatile uint8_t queue_tail = 0;
volatile uint8_t usb_mode = 1;
volatile uint8_t usb_active = 0;
volatile uint8_t pp_buf_ready[2] = {0, 0};
volatile uint8_t pp_current_buf = 0;
volatile uint8_t pp_next_buf = 0;
volatile uint16_t current_move_ticks_left = 0;
volatile uint16_t current_play_ticks_left = 0;
volatile uint8_t pingpong_mode_active = 0;

/* BSRR double buffers */
uint32_t bsrr_portF_0[PP_BUF_LEN];
uint32_t bsrr_portF_1[PP_BUF_LEN];
uint32_t bsrr_portG_0[PP_BUF_LEN];
uint32_t bsrr_portG_1[PP_BUF_LEN];
uint32_t bsrr_portC_0[PP_BUF_LEN];
uint32_t bsrr_portC_1[PP_BUF_LEN];

/* Direction masks for each buffer */
uint32_t pp_dir_setF[2], pp_dir_resetF[2];
uint32_t pp_dir_setG[2], pp_dir_resetG[2];
uint32_t pp_dir_setC[2], pp_dir_resetC[2];

/**
 * @brief  Queue a new 20ms move command.
 * @param  steps  Array of 6 step targets for this segment
 */
void DDA_Queue_Move(int16_t steps[NUM_AXES]) {
  uint8_t next_tail = (queue_tail + 1) % CMD_QUEUE_SIZE;
  if (next_tail != queue_head) {
    for (int i = 0; i < NUM_AXES; i++) {
      cmd_queue[queue_tail].steps[i] = steps[i];
    }
    queue_tail = next_tail;
  }
}

/**
 * @brief  Precompute a 5ms trajectory segment directly into one of the double buffers.
 *         Maintains the phase accumulator and step timer states across segment boundaries.
 * @param  buf_idx  Buffer index (0 or 1) to write to
 */
void DDA_PreCompute_PP_Chunk(uint8_t buf_idx) {
  uint32_t *bufF = (buf_idx == 0) ? bsrr_portF_0 : bsrr_portF_1;
  uint32_t *bufG = (buf_idx == 0) ? bsrr_portG_0 : bsrr_portG_1;
  uint32_t *bufC = (buf_idx == 0) ? bsrr_portC_0 : bsrr_portC_1;
  
  /* Prepare direction masks for this chunk */
  uint32_t dir_setF = 0, dir_resetF = 0;
  uint32_t dir_setG = 0, dir_resetG = 0;
  uint32_t dir_setC = 0, dir_resetC = 0;
  
  for (int i = 0; i < NUM_AXES; i++) {
    uint32_t pin = axes[i].dir_pin;
    if (axes[i].dir) {
      if (axes[i].dir_port == GPIOF) dir_resetF |= pin;
      else if (axes[i].dir_port == GPIOG) dir_resetG |= pin;
      else if (axes[i].dir_port == GPIOC) dir_resetC |= pin;
    } else {
      if (axes[i].dir_port == GPIOF) dir_setF |= pin;
      else if (axes[i].dir_port == GPIOG) dir_setG |= pin;
      else if (axes[i].dir_port == GPIOC) dir_setC |= pin;
    }
  }
  
  pp_dir_setF[buf_idx] = dir_setF;
  pp_dir_resetF[buf_idx] = dir_resetF;
  pp_dir_setG[buf_idx] = dir_setG;
  pp_dir_resetG[buf_idx] = dir_resetG;
  pp_dir_setC[buf_idx] = dir_setC;
  pp_dir_resetC[buf_idx] = dir_resetC;

  for (uint32_t tick = 0; tick < PP_TICKS; tick++) {
    uint32_t reset_maskF = 0;
    uint32_t set_maskF = 0;
    uint32_t reset_maskG = 0;
    uint32_t set_maskG = 0;
    uint32_t reset_maskC = 0;
    uint32_t set_maskC = 0;
    
    for (int i = 0; i < NUM_AXES; i++) {
      /* RESET logic (Active-High step pulses: reset pin to LOW) */
      if (axes[i].state_timer > 0) {
        axes[i].state_timer--;
        if (axes[i].state_timer == 0) {
          uint32_t reset_val = axes[i].step_pin; // SET HIGH at end of pulse
          if (axes[i].step_port == GPIOF)
            reset_maskF |= reset_val;
          else if (axes[i].step_port == GPIOG)
            reset_maskG |= reset_val;
          else if (axes[i].step_port == GPIOC)
            reset_maskC |= reset_val;
        }
      }
      /* DDA accumulator logic */
      if (axes[i].steps_done < axes[i].steps_total) {
        axes[i].accum += axes[i].velocity;
        if (axes[i].accum >= STEP_SCALE) {
          axes[i].accum -= STEP_SCALE;
          uint32_t set_val = (uint32_t)axes[i].step_pin << 16; // RESET LOW at start of pulse
          if (axes[i].step_port == GPIOF)
            set_maskF |= set_val;
          else if (axes[i].step_port == GPIOG)
            set_maskG |= set_val;
          else if (axes[i].step_port == GPIOC)
            set_maskC |= set_val;
          axes[i].state_timer = 1;
          axes[i].steps_done++;
        }
      }
    }
    
    /* Store the precomputed masks into the specific DMA buffers */
    bufF[2 * tick] = reset_maskF;
    bufF[2 * tick + 1] = set_maskF;
    bufG[2 * tick] = reset_maskG;
    bufG[2 * tick + 1] = set_maskG;
    bufC[2 * tick] = reset_maskC;
    bufC[2 * tick + 1] = set_maskC;
  }
}

/**
 * @brief  Configure DMA streams to play the selected buffer and start the motion segment.
 * @param  buffer_idx  Buffer index (0 or 1) to play
 */
void DDA_Start_PP(uint8_t buffer_idx) {
  uint32_t addrF = (buffer_idx == 0) ? (uint32_t)bsrr_portF_0 : (uint32_t)bsrr_portF_1;
  uint32_t addrG = (buffer_idx == 0) ? (uint32_t)bsrr_portG_0 : (uint32_t)bsrr_portG_1;
  uint32_t addrC = (buffer_idx == 0) ? (uint32_t)bsrr_portC_0 : (uint32_t)bsrr_portC_1;
  
  /* Set direction pins immediately for this buffer chunk */
  if (pp_dir_setF[buffer_idx]) GPIOF->BSRR = pp_dir_setF[buffer_idx];
  if (pp_dir_resetF[buffer_idx]) GPIOF->BSRR = (pp_dir_resetF[buffer_idx] << 16);
  if (pp_dir_setG[buffer_idx]) GPIOG->BSRR = pp_dir_setG[buffer_idx];
  if (pp_dir_resetG[buffer_idx]) GPIOG->BSRR = (pp_dir_resetG[buffer_idx] << 16);
  if (pp_dir_setC[buffer_idx]) GPIOC->BSRR = pp_dir_setC[buffer_idx];
  if (pp_dir_resetC[buffer_idx]) GPIOC->BSRR = (pp_dir_resetC[buffer_idx] << 16);
  
  /* Temporarily stop TIM1 and disable DMA streams */
  TIM1->CR1 &= ~TIM_CR1_CEN;
  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  DMA2_Stream1->CR &= ~DMA_SxCR_EN;
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream5->CR & DMA_SxCR_EN);
  while (DMA2_Stream1->CR & DMA_SxCR_EN);
  while (DMA2_Stream2->CR & DMA_SxCR_EN);
  
  /* Clear DMA flags */
  DMA2->LIFCR = (0x3D << 6) | (0x3D << 16); // Stream 1 and 2
  DMA2->HIFCR = (0x3D << 6);               // Stream 5
  
  /* Set memory addresses */
  DMA2_Stream5->M0AR = addrF;
  DMA2_Stream1->M0AR = addrG;
  DMA2_Stream2->M0AR = addrC;
  
  /* Set transfer counts */
  DMA2_Stream5->NDTR = PP_BUF_LEN;
  DMA2_Stream1->NDTR = PP_BUF_LEN;
  DMA2_Stream2->NDTR = PP_BUF_LEN;
  
  /* Re-enable DMA streams */
  DMA2_Stream5->CR |= DMA_SxCR_EN;
  DMA2_Stream1->CR |= DMA_SxCR_EN;
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  
  /* Start TIM1 */
  simulation_running = 1;
  TIM1->CNT = 0;
  TIM1->SR = 0;
  TIM1->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief  Main loop for the Ping-Pong Double-Buffering mode.
 */
void main_pingpong(void) {
  pingpong_mode_active = 1;
  uint32_t last_test_tick = 0;
  
  /* Enable all motors (EN LOW = active) */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].en_port->BSRR = (uint32_t)axes[i].en_pin << 16;
  }
  
  /* Set all step pins to HIGH (idle state for active-low) */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].step_port->BSRR = axes[i].step_pin;
  }
  
  /* LED ON initially */
  HAL_GPIO_WritePin(WORK_LED_GPIO_Port, WORK_LED_Pin, GPIO_PIN_SET);
  
  while (1) {
    /* If in USB mode and no USB activity yet, handle 10s timeout to fall back to test mode */
    if (usb_mode == 1 && !usb_active) {
      if (HAL_GetTick() > 10000) {
        usb_mode = 0;
        HAL_GPIO_WritePin(WORK_LED_GPIO_Port, WORK_LED_Pin, GPIO_PIN_RESET); // LED OFF in Test Mode
      }
    } else if (usb_active) {
      HAL_GPIO_WritePin(WORK_LED_GPIO_Port, WORK_LED_Pin, GPIO_PIN_SET); // LED ON in USB Mode
    }

    /* If in standalone test mode, periodically push the 20ms test move command */
    if (usb_mode == 0) {
      if (!simulation_running && (queue_head == queue_tail)) {
        if (HAL_GetTick() - last_test_tick >= 1000) {
          int16_t test_steps[NUM_AXES] = {1800, 2000, 3000, 2000, 500, 0};
          DDA_Queue_Move(test_steps);
          last_test_tick = HAL_GetTick();
          HAL_GPIO_TogglePin(WORK_LED_GPIO_Port, WORK_LED_Pin); // Toggle LED to show execution
        }
      }
    }
    
    /* Refill the inactive buffer if it is empty and we have data or are streaming */
    if (pp_buf_ready[pp_next_buf] == 0) {
      if (current_move_ticks_left == 0) {
        // Pop next move from queue
        if (queue_head != queue_tail) {
          MoveCommand_t cmd = cmd_queue[queue_head];
          queue_head = (queue_head + 1) % CMD_QUEUE_SIZE;
          
          for (int i = 0; i < NUM_AXES; i++) {
            int16_t steps_rel = cmd.steps[i];
            uint32_t abs_steps = (steps_rel < 0) ? -steps_rel : steps_rel;
            axes[i].steps_total = abs_steps;
            axes[i].steps_done = 0;
            axes[i].velocity = (abs_steps * STEP_SCALE + 3999U) / 4000U;
            axes[i].dir = (steps_rel < 0) ? 1 : 0;
          }
          current_move_ticks_left = 4000; // 20ms segment = 4000 ticks of 5us
        }
      }
      
      if (current_move_ticks_left > 0) {
        DDA_PreCompute_PP_Chunk(pp_next_buf);
        current_move_ticks_left -= 1000;
        pp_buf_ready[pp_next_buf] = 1;
        pp_next_buf = 1 - pp_next_buf;
      }
    }

    /* Start execution if the DMA is idle and the current buffer has data */
    if (!simulation_running) {
      if (pp_buf_ready[pp_current_buf]) {
        current_play_ticks_left = 4000; // Initialize play ticks
        DDA_Start_PP(pp_current_buf);
      }
    }
    
    HAL_Delay(1);
  }
}

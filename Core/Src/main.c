/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "sdio.h"
#include "usb_otg.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef *step_port;
  uint16_t step_pin;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *en_port;
  uint16_t en_pin;
  uint32_t accum;
  uint32_t velocity;
  uint32_t steps_total;
  uint32_t steps_done;
  uint8_t state_timer;
  uint8_t dir;
} AxisConfig_t;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_AXES 6
#define STEP_SCALE 65536U /* 16-bit DDA resolution */
#define DMA_TICKS 4000U /* Logical DDA ticks per motion segment (20ms @ 5µs) */
#define DMA_BUF_LEN (DMA_TICKS * 2) /* DMA entries: 2 per tick (RESET + SET phases) */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
AxisConfig_t axes[NUM_AXES];
volatile uint8_t simulation_running = 0;
uint32_t bsrr_portF[DMA_BUF_LEN];
uint32_t bsrr_portG[DMA_BUF_LEN];
uint32_t bsrr_portC[DMA_BUF_LEN];
volatile uint32_t dma_ticks_used = 0;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_TIM1_Init(void);
void DDA_Init(void);
void DDA_SetTarget(uint32_t target_steps[NUM_AXES]);
void DDA_Start(void);
void DMA_StepInit(void);
void DDA_PreCompute(void);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Initialize axes pin mapping for M0–M5 (STEP0–STEP5)
 */
void DDA_Init(void) {
  /* M0: STEP0=PF13, DIR0=PF12, EN0=PF14 */
  axes[0].step_port = STEP0_GPIO_Port;
  axes[0].step_pin = STEP0_Pin;
  axes[0].dir_port = DIR0_GPIO_Port;
  axes[0].dir_pin = DIR0_Pin;
  axes[0].en_port = EN0_GPIO_Port;
  axes[0].en_pin = EN0_Pin;
  /* M1: STEP1=PG0,  DIR1=PG1,  EN1=PF15 */
  axes[1].step_port = STEP1_GPIO_Port;
  axes[1].step_pin = STEP1_Pin;
  axes[1].dir_port = DIR1_GPIO_Port;
  axes[1].dir_pin = DIR1_Pin;
  axes[1].en_port = EN1_GPIO_Port;
  axes[1].en_pin = EN1_Pin;
  /* M2: STEP2=PF11, DIR2=PG3,  EN2=PG5 */
  axes[2].step_port = STEP2_GPIO_Port;
  axes[2].step_pin = STEP2_Pin;
  axes[2].dir_port = DIR2_GPIO_Port;
  axes[2].dir_pin = DIR2_Pin;
  axes[2].en_port = EN2_GPIO_Port;
  axes[2].en_pin = EN2_Pin;
  /* M3: STEP3=PG4,  DIR3=PC1,  EN3=PA0 */
  axes[3].step_port = STEP3_GPIO_Port;
  axes[3].step_pin = STEP3_Pin;
  axes[3].dir_port = DIR3_GPIO_Port;
  axes[3].dir_pin = DIR3_Pin;
  axes[3].en_port = EN3_GPIO_Port;
  axes[3].en_pin = EN3_Pin;
  /* M4: STEP4=PF9,  DIR4=PF10, EN4=PG2 */
  axes[4].step_port = STEP4_GPIO_Port;
  axes[4].step_pin = STEP4_Pin;
  axes[4].dir_port = DIR4_GPIO_Port;
  axes[4].dir_pin = DIR4_Pin;
  axes[4].en_port = EN4_GPIO_Port;
  axes[4].en_pin = EN4_Pin;
  /* M5: STEP5=PC13, DIR5=PF0,  EN5=PF1 */
  axes[5].step_port = STEP5_GPIO_Port;
  axes[5].step_pin = STEP5_Pin;
  axes[5].dir_port = DIR5_GPIO_Port;
  axes[5].dir_pin = DIR5_Pin;
  axes[5].en_port = EN5_GPIO_Port;
  axes[5].en_pin = EN5_Pin;
  /* Reset state for all axes */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].accum = 0;
    axes[i].velocity = 0;
    axes[i].steps_total = 0;
    axes[i].steps_done = 0;
    axes[i].state_timer = 0;
    axes[i].dir = 0;
  }
}
/**
 * @brief  Set target steps and compute DDA velocities for all axes.
 *         The dominant axis (max steps) runs at full speed (velocity =
 * STEP_SCALE). All others are proportionally scaled so they finish at the same
 * time.
 * @param  target_steps  Array of 6 absolute step counts
 */
void DDA_SetTarget(uint32_t target_steps[NUM_AXES]) {
  dma_ticks_used = 0;
  /* Find dominant axis (max steps) */
  uint32_t max_steps = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    if (target_steps[i] > max_steps)
      max_steps = target_steps[i];
  }
  if (max_steps == 0)
    return;
  /* Compute velocity for each axis to complete within a fixed 20ms duration
   * (4000 ticks of 5us) */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].steps_total = target_steps[i];
    axes[i].steps_done = 0;
    axes[i].accum = 0;
    axes[i].state_timer = 0;
    /* Use ceiling division to prevent rounding errors for small step counts */
    axes[i].velocity =
        (uint32_t)(((uint64_t)target_steps[i] * STEP_SCALE + 3999U) / 4000U);
  }
}
/**
 * @brief  Enable motors, set DIR forward, and start TIM2 DDA.
 */
/**
 * @brief  Enable motors, set DIR forward, and start TIM1 DDA.
 */
void DDA_Start(void) {
  if (dma_ticks_used == 0) {
    simulation_running = 0;
    return;
  }
  /* Enable all motors (EN LOW = active for A4988/TMC drivers) */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].en_port->BSRR = (uint32_t)axes[i].en_pin << 16; /* EN LOW */
  }
  /* Set direction forward (DIR HIGH) */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].dir_port->BSRR = axes[i].dir_pin; /* DIR HIGH = forward */
  }
  /* Brief delay for DIR setup time (>650ns for A4988) */
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  /* Configure DMA transfer counts and re-enable streams */

  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  DMA2_Stream1->CR &= ~DMA_SxCR_EN;
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;

  while (DMA2_Stream5->CR & DMA_SxCR_EN);
  while (DMA2_Stream1->CR & DMA_SxCR_EN);
  while (DMA2_Stream2->CR & DMA_SxCR_EN);
  /* Clear DMA flags */

  DMA2->LIFCR = (0x3D << 6) | (0x3D << 16); // Stream 1 and 2
  DMA2->HIFCR = (0x3D << 6);               // Stream 5
  /* Set number of transfers */

  DMA2_Stream5->NDTR = dma_ticks_used * 2;
  DMA2_Stream1->NDTR = dma_ticks_used * 2;
  DMA2_Stream2->NDTR = dma_ticks_used * 2;
  /* Enable DMA streams */
  DMA1_Stream1->CR |= DMA_SxCR_EN;
  DMA1_Stream5->CR |= DMA_SxCR_EN;
  DMA1_Stream6->CR |= DMA_SxCR_EN;
  DMA2_Stream5->CR |= DMA_SxCR_EN;
  DMA2_Stream1->CR |= DMA_SxCR_EN;
  DMA2_Stream2->CR |= DMA_SxCR_EN;

  /* Start TIM1 */
  simulation_running = 1;

  TIM1->CNT = 0;
  TIM1->SR = 0;
  TIM1->CR1 |= TIM_CR1_CEN;
}
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
  /* USER CODE BEGIN SysInit */
  // Force SysTick recalibrate — dùng HAL_RCC_GetSysClockFreq() trực tiếp
  SystemCoreClock = HAL_RCC_GetSysClockFreq();
  HAL_SYSTICK_Config(SystemCoreClock / 1000U);
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // Bỏ qua các ngoại vi không cần thiết cho test — tránh treo MCU
  // MX_CAN1_Init();
  // MX_SDIO_SD_Init();
  // MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM1_Init();
  DDA_Init();
  DMA_StepInit();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* Phase 3 Test — 6 axes synchronized DDA
     * M0=1000, M1=2000, M2=3000, M3=4000(dominant), M4=500, M5=0 */
    uint32_t test_steps[NUM_AXES] = {1800, 2000, 3000, 2000, 500, 0};
    DDA_SetTarget(test_steps);
    DDA_PreCompute();
    DDA_Start();
    /* Wait for all axes to complete */
    while (simulation_running) {
      /* Busy wait */
    }
    /* Toggle WORK_LED to show completion */
    HAL_GPIO_TogglePin(WORK_LED_GPIO_Port, WORK_LED_Pin);
    /* Delay 1 second before next run */
    HAL_Delay(1000);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    // Nếu HSE không khởi động được → bỏ qua, MCU chạy bằng HSI 16MHz
    return;
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    return; // Bỏ qua, không treo
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    return; // Bỏ qua, không treo
  }
}
/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_SDIO | RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 6;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    // Bỏ qua, không treo
  }
}
/* USER CODE BEGIN 4 */

void MX_TIM1_Init(void) {
  /* 1. Enable TIM1 clock in APB2 */
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  /* 2. Prescaler: APB2 Timer = 180MHz → 2MHz count clock */
  TIM1->PSC = 90 - 1;
  /* 3. Auto-Reload: 2MHz / 5 = 400kHz period (2.5us) */

  TIM1->ARR = 5 - 1;
  /* 4. Configure CCR1 and CCR2 for Compare events */

  TIM1->CCR1 = 1;
  TIM1->CCR2 = 2;
  /* 5. Force update to load PSC and ARR shadow registers */
  TIM1->EGR = TIM_EGR_UG;
  /* 6. Clear update flag and other flags */
  TIM1->SR = 0;
  /* 7. Enable DMA requests: Update (UDE), CC1 (CC1DE), CC2 (CC2DE) */
  TIM1->DIER |= (TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE);
  /* 8. Enable CC1 and CC2 outputs internally (required for compare event DMA trigger) */
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
  /* 9. Set MOE bit in BDTR to enable main outputs for advanced timer */
  TIM1->BDTR |= TIM_BDTR_MOE;
}
void DMA_StepInit(void) {
  /* 1. Enable DMA1 clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  /* 1. Enable DMA2 clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  /* Delay after clock enabling */
  __asm("nop");
  __asm("nop");
  /* 2. Disable DMA streams to allow configuration */

  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  DMA2_Stream1->CR &= ~DMA_SxCR_EN;
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;
  /* Wait for streams to be disabled */

  while (DMA2_Stream5->CR & DMA_SxCR_EN);
  while (DMA2_Stream1->CR & DMA_SxCR_EN);
  while (DMA2_Stream2->CR & DMA_SxCR_EN);
  /* 3. Set Peripheral Address (PAR) to GPIO BSRR registers */

  DMA2_Stream5->PAR = (uint32_t)&(GPIOF->BSRR);
  DMA2_Stream1->PAR = (uint32_t)&(GPIOG->BSRR);
  DMA2_Stream2->PAR = (uint32_t)&(GPIOC->BSRR);
  /* 4. Set Memory Address (M0AR) to our precomputed buffers */

  DMA2_Stream5->M0AR = (uint32_t)bsrr_portF;
  DMA2_Stream1->M0AR = (uint32_t)bsrr_portG;
  DMA2_Stream2->M0AR = (uint32_t)bsrr_portC;
  /* 5. Configure Control Registers (CR):
   * - CHSEL: Channel 3 (3U << 25)
   * - CHSEL: Channel 6 (6U << 25)
   * - PL: Priority Level High (2U << 16)
   * - MSIZE: 32-bit (2U << 13)
   * - PSIZE: 32-bit (2U << 11)
   * - MINC: Memory increment enabled (DMA_SxCR_MINC)
   * - DIR: Memory-to-peripheral (1U << 6)
   */

  DMA2_Stream5->CR = (6U << 25) | (2U << 16) | (2U << 13) | (2U << 11) | DMA_SxCR_MINC | (1U << 6) | DMA_SxCR_TCIE;
  DMA2_Stream1->CR = (6U << 25) | (2U << 16) | (2U << 13) | (2U << 11) | DMA_SxCR_MINC | (1U << 6);
  DMA2_Stream2->CR = (6U << 25) | (2U << 16) | (2U << 13) | (2U << 11) | DMA_SxCR_MINC | (1U << 6);
  /* 6. Ensure Direct Mode is used (DMDIS = 0 in FCR) */

  DMA2_Stream5->FCR &= ~DMA_SxFCR_DMDIS;
  DMA2_Stream1->FCR &= ~DMA_SxFCR_DMDIS;
  DMA2_Stream2->FCR &= ~DMA_SxFCR_DMDIS;

  /* 7. Configure NVIC for Stream 5 Transfer Complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}
void DDA_PreCompute(void) {
  /* Copy axes configuration to a local copy to run simulation offline without modifying actual steps_done */
  AxisConfig_t local_axes[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    local_axes[i] = axes[i];
  }
  uint32_t tick = 0;
  for (tick = 0; tick < DMA_TICKS; tick++) {
    uint8_t all_done = 1;
    uint32_t reset_maskF = 0;
    uint32_t set_maskF = 0;
    uint32_t reset_maskG = 0;
    uint32_t set_maskG = 0;
    uint32_t reset_maskC = 0;
    uint32_t set_maskC = 0;
    for (int i = 0; i < NUM_AXES; i++) {
      /* RESET logic */
      if (local_axes[i].state_timer > 0) {
        local_axes[i].state_timer--;
        if (local_axes[i].state_timer == 0) {
          uint32_t reset_val = (uint32_t)local_axes[i].step_pin << 16;
          if (local_axes[i].step_port == GPIOF) reset_maskF |= reset_val;
          else if (local_axes[i].step_port == GPIOG) reset_maskG |= reset_val;
          else if (local_axes[i].step_port == GPIOC) reset_maskC |= reset_val;
        }
      }
      /* DDA accumulator logic */
      if (local_axes[i].steps_done < local_axes[i].steps_total) {
        all_done = 0;
        local_axes[i].accum += local_axes[i].velocity;
        if (local_axes[i].accum >= STEP_SCALE) {
          local_axes[i].accum -= STEP_SCALE;
          uint32_t set_val = local_axes[i].step_pin;
          if (local_axes[i].step_port == GPIOF) set_maskF |= set_val;
          else if (local_axes[i].step_port == GPIOG) set_maskG |= set_val;
          else if (local_axes[i].step_port == GPIOC) set_maskC |= set_val;
          local_axes[i].state_timer = 1;
          local_axes[i].steps_done++;
        }
      }
    }
    /* Store the precomputed masks into the DMA buffers */
    bsrr_portF[2 * tick] = reset_maskF;
    bsrr_portF[2 * tick + 1] = set_maskF;
    bsrr_portG[2 * tick] = reset_maskG;
    bsrr_portG[2 * tick + 1] = set_maskG;
    bsrr_portC[2 * tick] = reset_maskC;
    bsrr_portC[2 * tick + 1] = set_maskC;
    /* Check if we can terminate early */
    if (all_done) {
      uint8_t timers_active = 0;
      for (int i = 0; i < NUM_AXES; i++) {
        if (local_axes[i].state_timer > 0) {
          timers_active = 1;
          break;
        }
      }
      if (!timers_active) {
        tick++; // Include the current tick
        break;
      }
    }
  }
  if (tick > DMA_TICKS) {
    tick = DMA_TICKS;
  }
  dma_ticks_used = tick;
  /* Update the main axes steps_done since we simulated the whole trajectory */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].steps_done = local_axes[i].steps_done;
    axes[i].accum = local_axes[i].accum;
    axes[i].state_timer = local_axes[i].state_timer;
  }
}
/* USER CODE END 4 */
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

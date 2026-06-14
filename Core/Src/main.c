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
#include "usb_device.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_TICKS 1000U /* Logical DDA ticks per motion segment (5ms @ 5µs) */
#define DMA_BUF_LEN (DMA_TICKS * 2) /* DMA entries: 2 per tick (RESET + SET phases) */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
AxisConfig_t axes[NUM_AXES];
volatile uint8_t simulation_running = 0;
/* Dynamic sub-segment buffers and USB variables are defined in main_pingpong.c */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_TIM1_Init(void);
void DDA_Init(void);
void DMA_StepInit(void);
uint8_t USB_IsConnected(void);
void CDC_Parse_Command(uint8_t *buf, uint32_t len);
void CDC_Send_Status(void);
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
  /* Reset state for all axes and set step pins to HIGH (idle state for active-low) */
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].accum = 0;
    axes[i].velocity = 0;
    axes[i].steps_total = 0;
    axes[i].steps_done = 0;
    axes[i].state_timer = 0;
    axes[i].dir = 0;
    axes[i].step_port->BSRR = axes[i].step_pin; // Set step pin HIGH (idle)
  }
}
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
  /* Force SysTick recalibrate */
  SystemCoreClock = HAL_RCC_GetSysClockFreq();
  HAL_SYSTICK_Config(SystemCoreClock / 1000U);
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  DDA_Init();
  DMA_StepInit();
  
  extern void main_pingpong(void);
  main_pingpong();
  /* USER CODE END 2 */
  while (1) {
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
  /* 8. Enable CC1 and CC2 outputs internally (required for compare event DMA
   * trigger) */
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

  while (DMA2_Stream5->CR & DMA_SxCR_EN)
    ;
  while (DMA2_Stream1->CR & DMA_SxCR_EN)
    ;
  while (DMA2_Stream2->CR & DMA_SxCR_EN)
    ;
  /* 3. Set Peripheral Address (PAR) to GPIO BSRR registers */

  DMA2_Stream5->PAR = (uint32_t)&(GPIOF->BSRR);
  DMA2_Stream1->PAR = (uint32_t)&(GPIOG->BSRR);
  DMA2_Stream2->PAR = (uint32_t)&(GPIOC->BSRR);
  /* 4. Set Memory Address (M0AR) to our precomputed buffers */
  extern uint32_t bsrr_portF_0[];
  extern uint32_t bsrr_portG_0[];
  extern uint32_t bsrr_portC_0[];
  DMA2_Stream5->M0AR = (uint32_t)bsrr_portF_0;
  DMA2_Stream1->M0AR = (uint32_t)bsrr_portG_0;
  DMA2_Stream2->M0AR = (uint32_t)bsrr_portC_0;
  /* 5. Configure Control Registers (CR):
   * - CHSEL: Channel 3 (3U << 25)
   * - CHSEL: Channel 6 (6U << 25)
   * - PL: Priority Level High (2U << 16)
   * - MSIZE: 32-bit (2U << 13)
   * - PSIZE: 32-bit (2U << 11)
   * - MINC: Memory increment enabled (DMA_SxCR_MINC)
   * - DIR: Memory-to-peripheral (1U << 6)
   */

  DMA2_Stream5->CR = (6U << 25) | (2U << 16) | (2U << 13) | (2U << 11) |
                     DMA_SxCR_MINC | (1U << 6) | DMA_SxCR_TCIE;
  DMA2_Stream1->CR = (6U << 25) | (2U << 16) | (2U << 13) | (2U << 11) |
                     DMA_SxCR_MINC | (1U << 6);
  DMA2_Stream2->CR = (6U << 25) | (2U << 16) | (2U << 13) | (2U << 11) |
                     DMA_SxCR_MINC | (1U << 6);
  /* 6. Ensure Direct Mode is used (DMDIS = 0 in FCR) */

  DMA2_Stream5->FCR &= ~DMA_SxFCR_DMDIS;
  DMA2_Stream1->FCR &= ~DMA_SxFCR_DMDIS;
  DMA2_Stream2->FCR &= ~DMA_SxFCR_DMDIS;

  /* 7. Configure NVIC for Stream 5 Transfer Complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}
/* DDA_PreCompute has been replaced by the streaming ping-pong precompute in main_pingpong.c */

uint8_t USB_IsConnected(void) {
  extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY) {
    USB_OTG_DeviceTypeDef *device =
        (USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE);
    if ((device->DSTS & USB_OTG_DSTS_SUSPSTS) == 0) {
      return 1;
    }
  }
  return 0;
}

__attribute__((weak)) uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len) {
  return 0; // Weak stub: return OK if VCP driver is not generated yet
}

void CDC_Send_Status(void) {
  uint8_t tx_buf[30];
  tx_buf[0] = 0xAA;
  tx_buf[1] = 'S';
  tx_buf[2] = simulation_running ? 0x01 : 0x00;

  // Pack steps_done for 6 axes as big-endian 32-bit integers
  for (int i = 0; i < NUM_AXES; i++) {
    uint32_t sd = axes[i].steps_done;
    tx_buf[3 + i * 4] = (uint8_t)(sd >> 24);
    tx_buf[3 + i * 4 + 1] = (uint8_t)(sd >> 16);
    tx_buf[3 + i * 4 + 2] = (uint8_t)(sd >> 8);
    tx_buf[3 + i * 4 + 3] = (uint8_t)sd;
  }
  tx_buf[27] = 0x0A; // EOL

  CDC_Transmit_FS(tx_buf, 28);
}

void CDC_Parse_Command(uint8_t *buf, uint32_t len) {
  if (len < 3 || buf[0] != 0xAA)
    return; // Check SOF

  usb_active = 1; // Mark USB as active

  if (usb_mode == 0) {
    /* Abort test mode and transition to USB mode */
    usb_mode = 1;
    extern volatile uint8_t queue_head;
    extern volatile uint8_t queue_tail;
    extern volatile uint8_t pp_buf_ready[2];
    extern volatile uint16_t current_move_ticks_left;
    extern volatile uint16_t current_play_ticks_left;
    
    TIM1->CR1 &= ~TIM_CR1_CEN;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    simulation_running = 0;
    
    queue_head = 0;
    queue_tail = 0;
    pp_buf_ready[0] = 0;
    pp_buf_ready[1] = 0;
    current_move_ticks_left = 0;
    current_play_ticks_left = 0;
  }

  if (buf[1] == 'M') { // Move command
    if (len < 15) return;
    int16_t steps[NUM_AXES];
    for (int i = 0; i < NUM_AXES; i++) {
      steps[i] = (int16_t)((buf[2 + i * 2] << 8) | buf[3 + i * 2]);
    }

    extern void DDA_Queue_Move(int16_t steps[NUM_AXES]);
    extern volatile uint8_t queue_head;
    extern volatile uint8_t queue_tail;
    
    // Check if queue is full
    uint8_t next_tail = (queue_tail + 1) % CMD_QUEUE_SIZE;
    if (next_tail != queue_head) {
      DDA_Queue_Move(steps);
      uint8_t ack = 'K'; // Queued OK
      CDC_Transmit_FS(&ack, 1);
    } else {
      uint8_t nack = 'N'; // Queue full
      CDC_Transmit_FS(&nack, 1);
    }
  } else if (buf[1] == 'E') { // Emergency Stop
    TIM1->CR1 &= ~TIM_CR1_CEN;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    simulation_running = 0;
    
    extern volatile uint8_t queue_head;
    extern volatile uint8_t queue_tail;
    extern volatile uint8_t pp_buf_ready[2];
    extern volatile uint16_t current_move_ticks_left;
    extern volatile uint16_t current_play_ticks_left;
    
    queue_head = 0;
    queue_tail = 0;
    pp_buf_ready[0] = 0;
    pp_buf_ready[1] = 0;
    current_move_ticks_left = 0;
    current_play_ticks_left = 0;
    
    for (int i = 0; i < NUM_AXES; i++) {
      axes[i].step_port->BSRR = axes[i].step_pin; // Reset step pins to HIGH (idle)
    }
  } else if (buf[1] == 'S') { // Status Query
    CDC_Send_Status();
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

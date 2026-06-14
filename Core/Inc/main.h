/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define NUM_AXES 6
#define STEP_SCALE 65536U
#define PP_TICKS 1000U
#define PP_BUF_LEN (PP_TICKS * 2)

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

typedef struct {
  int16_t steps[NUM_AXES];
} MoveCommand_t;

#define CMD_QUEUE_SIZE 4
extern MoveCommand_t cmd_queue[CMD_QUEUE_SIZE];
extern volatile uint8_t queue_head;
extern volatile uint8_t queue_tail;
extern volatile uint8_t usb_mode;
extern volatile uint8_t usb_active;
extern volatile uint8_t pp_buf_ready[2];
extern volatile uint8_t pp_current_buf;
extern volatile uint8_t pp_next_buf;
extern volatile uint16_t current_move_ticks_left;
extern volatile uint16_t current_play_ticks_left;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEP6_Pin GPIO_PIN_2
#define STEP6_GPIO_Port GPIOE
#define DIR6_Pin GPIO_PIN_3
#define DIR6_GPIO_Port GPIOE
#define CS5_Pin GPIO_PIN_4
#define CS5_GPIO_Port GPIOE
#define FAN1_Pin GPIO_PIN_5
#define FAN1_GPIO_Port GPIOE
#define STEP7_Pin GPIO_PIN_6
#define STEP7_GPIO_Port GPIOE
#define STEP5_Pin GPIO_PIN_13
#define STEP5_GPIO_Port GPIOC
#define DET_Pin GPIO_PIN_14
#define DET_GPIO_Port GPIOC
#define PC15_Pin GPIO_PIN_15
#define PC15_GPIO_Port GPIOC
#define DIR5_Pin GPIO_PIN_0
#define DIR5_GPIO_Port GPIOF
#define EN5_Pin GPIO_PIN_1
#define EN5_GPIO_Port GPIOF
#define CS4_Pin GPIO_PIN_2
#define CS4_GPIO_Port GPIOF
#define TB_Pin GPIO_PIN_3
#define TB_GPIO_Port GPIOF
#define T0_Pin GPIO_PIN_4
#define T0_GPIO_Port GPIOF
#define T1_Pin GPIO_PIN_5
#define T1_GPIO_Port GPIOF
#define T2_Pin GPIO_PIN_6
#define T2_GPIO_Port GPIOF
#define T3_Pin GPIO_PIN_7
#define T3_GPIO_Port GPIOF
#define MAX_CS_Pin GPIO_PIN_8
#define MAX_CS_GPIO_Port GPIOF
#define STEP4_Pin GPIO_PIN_9
#define STEP4_GPIO_Port GPIOF
#define DIR4_Pin GPIO_PIN_10
#define DIR4_GPIO_Port GPIOF
#define PWR_DET_Pin GPIO_PIN_0
#define PWR_DET_GPIO_Port GPIOC
#define DIR3_Pin GPIO_PIN_1
#define DIR3_GPIO_Port GPIOC
#define EN3_Pin GPIO_PIN_0
#define EN3_GPIO_Port GPIOA
#define BED_OUT_Pin GPIO_PIN_1
#define BED_OUT_GPIO_Port GPIOA
#define HE0_Pin GPIO_PIN_2
#define HE0_GPIO_Port GPIOA
#define HE1_Pin GPIO_PIN_3
#define HE1_GPIO_Port GPIOA
#define CS0_Pin GPIO_PIN_4
#define CS0_GPIO_Port GPIOC
#define KILL_Pin GPIO_PIN_5
#define KILL_GPIO_Port GPIOC
#define RGB_Pin GPIO_PIN_0
#define RGB_GPIO_Port GPIOB
#define PB1_Pin GPIO_PIN_1
#define PB1_GPIO_Port GPIOB
#define PB2_Pin GPIO_PIN_2
#define PB2_GPIO_Port GPIOB
#define STEP2_Pin GPIO_PIN_11
#define STEP2_GPIO_Port GPIOF
#define DIR0_Pin GPIO_PIN_12
#define DIR0_GPIO_Port GPIOF
#define STEP0_Pin GPIO_PIN_13
#define STEP0_GPIO_Port GPIOF
#define EN0_Pin GPIO_PIN_14
#define EN0_GPIO_Port GPIOF
#define EN1_Pin GPIO_PIN_15
#define EN1_GPIO_Port GPIOF
#define STEP1_Pin GPIO_PIN_0
#define STEP1_GPIO_Port GPIOG
#define DIR1_Pin GPIO_PIN_1
#define DIR1_GPIO_Port GPIOG
#define PE7_Pin GPIO_PIN_7
#define PE7_GPIO_Port GPIOE
#define PE8_Pin GPIO_PIN_8
#define PE8_GPIO_Port GPIOE
#define PE9_Pin GPIO_PIN_9
#define PE9_GPIO_Port GPIOE
#define PE10_Pin GPIO_PIN_10
#define PE10_GPIO_Port GPIOE
#define PS_ON_Pin GPIO_PIN_11
#define PS_ON_GPIO_Port GPIOE
#define PE12_Pin GPIO_PIN_12
#define PE12_GPIO_Port GPIOE
#define PE13_Pin GPIO_PIN_13
#define PE13_GPIO_Port GPIOE
#define PE14_Pin GPIO_PIN_14
#define PE14_GPIO_Port GPIOE
#define PE15_Pin GPIO_PIN_15
#define PE15_GPIO_Port GPIOE
#define HE2_Pin GPIO_PIN_10
#define HE2_GPIO_Port GPIOB
#define HE3_Pin GPIO_PIN_11
#define HE3_GPIO_Port GPIOB
#define PD10_Pin GPIO_PIN_10
#define PD10_GPIO_Port GPIOD
#define CS1_Pin GPIO_PIN_11
#define CS1_GPIO_Port GPIOD
#define FAN2_Pin GPIO_PIN_12
#define FAN2_GPIO_Port GPIOD
#define FAN3_Pin GPIO_PIN_13
#define FAN3_GPIO_Port GPIOD
#define FAN4_Pin GPIO_PIN_14
#define FAN4_GPIO_Port GPIOD
#define FAN5_Pin GPIO_PIN_15
#define FAN5_GPIO_Port GPIOD
#define EN4_Pin GPIO_PIN_2
#define EN4_GPIO_Port GPIOG
#define DIR2_Pin GPIO_PIN_3
#define DIR2_GPIO_Port GPIOG
#define STEP3_Pin GPIO_PIN_4
#define STEP3_GPIO_Port GPIOG
#define EN2_Pin GPIO_PIN_5
#define EN2_GPIO_Port GPIOG
#define STOP_0_Pin GPIO_PIN_6
#define STOP_0_GPIO_Port GPIOG
#define PG7_Pin GPIO_PIN_7
#define PG7_GPIO_Port GPIOG
#define PG8_Pin GPIO_PIN_8
#define PG8_GPIO_Port GPIOG
#define CS2_Pin GPIO_PIN_6
#define CS2_GPIO_Port GPIOC
#define CS3_Pin GPIO_PIN_7
#define CS3_GPIO_Port GPIOC
#define FAN0_Pin GPIO_PIN_8
#define FAN0_GPIO_Port GPIOA
#define TFT_TX_Pin GPIO_PIN_9
#define TFT_TX_GPIO_Port GPIOA
#define TFT_RX_Pin GPIO_PIN_10
#define TFT_RX_GPIO_Port GPIOA
#define WORK_LED_Pin GPIO_PIN_13
#define WORK_LED_GPIO_Port GPIOA
#define DIR7_Pin GPIO_PIN_14
#define DIR7_GPIO_Port GPIOA
#define CS7_Pin GPIO_PIN_3
#define CS7_GPIO_Port GPIOD
#define EN6_Pin GPIO_PIN_4
#define EN6_GPIO_Port GPIOD
#define RAS_TX_Pin GPIO_PIN_5
#define RAS_TX_GPIO_Port GPIOD
#define RAS_RX_Pin GPIO_PIN_6
#define RAS_RX_GPIO_Port GPIOD
#define PD7_Pin GPIO_PIN_7
#define PD7_GPIO_Port GPIOD
#define STOP_1_Pin GPIO_PIN_9
#define STOP_1_GPIO_Port GPIOG
#define STOP_2_Pin GPIO_PIN_10
#define STOP_2_GPIO_Port GPIOG
#define STOP_3_Pin GPIO_PIN_11
#define STOP_3_GPIO_Port GPIOG
#define STOP_4_Pin GPIO_PIN_12
#define STOP_4_GPIO_Port GPIOG
#define STOP_5_Pin GPIO_PIN_13
#define STOP_5_GPIO_Port GPIOG
#define STOP_6_Pin GPIO_PIN_14
#define STOP_6_GPIO_Port GPIOG
#define STOP_7_Pin GPIO_PIN_15
#define STOP_7_GPIO_Port GPIOG
#define BLTOUCH_Pin GPIO_PIN_6
#define BLTOUCH_GPIO_Port GPIOB
#define BLTRIGG_Pin GPIO_PIN_7
#define BLTRIGG_GPIO_Port GPIOB
#define EE_SCL_Pin GPIO_PIN_8
#define EE_SCL_GPIO_Port GPIOB
#define EE_SDA_Pin GPIO_PIN_9
#define EE_SDA_GPIO_Port GPIOB
#define EN7_Pin GPIO_PIN_0
#define EN7_GPIO_Port GPIOE
#define CS6_Pin GPIO_PIN_1
#define CS6_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

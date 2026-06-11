/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PC2   ------> SPI2_MISO
     PC3   ------> SPI2_MOSI
     PA4   ------> SPI1_NSS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB12   ------> SPI2_NSS
     PB13   ------> SPI2_SCK
     PB14   ------> USB_OTG_HS_DM
     PB15   ------> USB_OTG_HS_DP
     PD8   ------> USART3_TX
     PD9   ------> USART3_RX
     PA9   ------> USART1_TX
     PA10   ------> USART1_RX
     PA15   ------> SPI3_NSS
     PD5   ------> USART2_TX
     PD6   ------> USART2_RX
     PB3   ------> SPI3_SCK
     PB4   ------> SPI3_MISO
     PB5   ------> SPI3_MOSI
     PB8   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STEP6_Pin|DIR6_Pin|CS5_Pin|FAN1_Pin
                          |STEP7_Pin|PS_ON_Pin|EN7_Pin|CS6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STEP5_Pin|DIR3_Pin|CS0_Pin|CS2_Pin
                          |CS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DIR5_Pin|EN5_Pin|CS4_Pin|MAX_CS_Pin
                          |STEP4_Pin|DIR4_Pin|STEP2_Pin|DIR0_Pin
                          |STEP0_Pin|EN0_Pin|EN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN3_Pin|BED_OUT_Pin|HE0_Pin|HE1_Pin
                          |FAN0_Pin|WORK_LED_Pin|DIR7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB_Pin|HE2_Pin|HE3_Pin|BLTOUCH_Pin
                          |BLTRIGG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, STEP1_Pin|DIR1_Pin|EN4_Pin|DIR2_Pin
                          |STEP3_Pin|EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CS1_Pin|FAN2_Pin|FAN3_Pin|FAN4_Pin
                          |FAN5_Pin|CS7_Pin|EN6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP6_Pin DIR6_Pin CS5_Pin FAN1_Pin
                           STEP7_Pin PS_ON_Pin EN7_Pin CS6_Pin */
  GPIO_InitStruct.Pin = STEP6_Pin|DIR6_Pin|CS5_Pin|FAN1_Pin
                          |STEP7_Pin|PS_ON_Pin|EN7_Pin|CS6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP5_Pin DIR3_Pin CS0_Pin CS2_Pin
                           CS3_Pin */
  GPIO_InitStruct.Pin = STEP5_Pin|DIR3_Pin|CS0_Pin|CS2_Pin
                          |CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DET_Pin PC15_Pin PWR_DET_Pin KILL_Pin */
  GPIO_InitStruct.Pin = DET_Pin|PC15_Pin|PWR_DET_Pin|KILL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR5_Pin EN5_Pin CS4_Pin MAX_CS_Pin
                           STEP4_Pin DIR4_Pin STEP2_Pin DIR0_Pin
                           STEP0_Pin EN0_Pin EN1_Pin */
  GPIO_InitStruct.Pin = DIR5_Pin|EN5_Pin|CS4_Pin|MAX_CS_Pin
                          |STEP4_Pin|DIR4_Pin|STEP2_Pin|DIR0_Pin
                          |STEP0_Pin|EN0_Pin|EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TB_Pin T0_Pin T1_Pin T2_Pin
                           T3_Pin */
  GPIO_InitStruct.Pin = TB_Pin|T0_Pin|T1_Pin|T2_Pin
                          |T3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN3_Pin BED_OUT_Pin HE0_Pin HE1_Pin
                           FAN0_Pin WORK_LED_Pin DIR7_Pin */
  GPIO_InitStruct.Pin = EN3_Pin|BED_OUT_Pin|HE0_Pin|HE1_Pin
                          |FAN0_Pin|WORK_LED_Pin|DIR7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_Pin HE2_Pin HE3_Pin BLTOUCH_Pin
                           BLTRIGG_Pin */
  GPIO_InitStruct.Pin = RGB_Pin|HE2_Pin|HE3_Pin|BLTOUCH_Pin
                          |BLTRIGG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1_Pin PB2_Pin */
  GPIO_InitStruct.Pin = PB1_Pin|PB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP1_Pin DIR1_Pin EN4_Pin DIR2_Pin
                           STEP3_Pin EN2_Pin */
  GPIO_InitStruct.Pin = STEP1_Pin|DIR1_Pin|EN4_Pin|DIR2_Pin
                          |STEP3_Pin|EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7_Pin PE8_Pin PE9_Pin PE10_Pin
                           PE12_Pin PE13_Pin PE14_Pin PE15_Pin */
  GPIO_InitStruct.Pin = PE7_Pin|PE8_Pin|PE9_Pin|PE10_Pin
                          |PE12_Pin|PE13_Pin|PE14_Pin|PE15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10_Pin PD7_Pin */
  GPIO_InitStruct.Pin = PD10_Pin|PD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin FAN2_Pin FAN3_Pin FAN4_Pin
                           FAN5_Pin CS7_Pin EN6_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|FAN2_Pin|FAN3_Pin|FAN4_Pin
                          |FAN5_Pin|CS7_Pin|EN6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STOP_0_Pin PG7_Pin PG8_Pin STOP_1_Pin
                           STOP_2_Pin STOP_3_Pin STOP_4_Pin STOP_5_Pin
                           STOP_6_Pin STOP_7_Pin */
  GPIO_InitStruct.Pin = STOP_0_Pin|PG7_Pin|PG8_Pin|STOP_1_Pin
                          |STOP_2_Pin|STOP_3_Pin|STOP_4_Pin|STOP_5_Pin
                          |STOP_6_Pin|STOP_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_TX_Pin TFT_RX_Pin */
  GPIO_InitStruct.Pin = TFT_TX_Pin|TFT_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RAS_TX_Pin RAS_RX_Pin */
  GPIO_InitStruct.Pin = RAS_TX_Pin|RAS_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EE_SCL_Pin EE_SDA_Pin */
  GPIO_InitStruct.Pin = EE_SCL_Pin|EE_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, B0_Pin|B1_Pin|B2_Pin|B3_Pin
                          |B4_Pin|B5_Pin|B6_Pin|B7_Pin
                          |SYNC_PIXEL_Pin|SYNC_ROW_Pin|SYNC_FRAME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin|R5_Pin|R6_Pin|R7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G0_Pin|G1_Pin|G2_Pin|G3_Pin
                          |G4_Pin|G5_Pin|G6_Pin|G7_Pin
                          |WR_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B0_Pin B1_Pin B2_Pin B3_Pin
                           B4_Pin B5_Pin B6_Pin B7_Pin
                           SYNC_PIXEL_Pin */
  GPIO_InitStruct.Pin = B0_Pin|B1_Pin|B2_Pin|B3_Pin
                          |B4_Pin|B5_Pin|B6_Pin|B7_Pin
                          |SYNC_PIXEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin R3_Pin
                           R4_Pin R5_Pin R6_Pin R7_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin|R5_Pin|R6_Pin|R7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : G0_Pin G1_Pin G2_Pin G3_Pin
                           G4_Pin G5_Pin G6_Pin G7_Pin
                           WR_Pin CS_Pin */
  GPIO_InitStruct.Pin = G0_Pin|G1_Pin|G2_Pin|G3_Pin
                          |G4_Pin|G5_Pin|G6_Pin|G7_Pin
                          |WR_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SYNC_ROW_Pin SYNC_FRAME_Pin */
  GPIO_InitStruct.Pin = SYNC_ROW_Pin|SYNC_FRAME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SYNC_Pin */
  GPIO_InitStruct.Pin = SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SYNC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

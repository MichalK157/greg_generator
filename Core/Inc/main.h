/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define B0_Pin GPIO_PIN_0
#define B0_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_1
#define B1_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_2
#define B2_GPIO_Port GPIOC
#define B3_Pin GPIO_PIN_3
#define B3_GPIO_Port GPIOC
#define R0_Pin GPIO_PIN_0
#define R0_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_1
#define R1_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_2
#define R2_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_3
#define R3_GPIO_Port GPIOA
#define R4_Pin GPIO_PIN_4
#define R4_GPIO_Port GPIOA
#define R5_Pin GPIO_PIN_5
#define R5_GPIO_Port GPIOA
#define R6_Pin GPIO_PIN_6
#define R6_GPIO_Port GPIOA
#define R7_Pin GPIO_PIN_7
#define R7_GPIO_Port GPIOA
#define B4_Pin GPIO_PIN_4
#define B4_GPIO_Port GPIOC
#define B5_Pin GPIO_PIN_5
#define B5_GPIO_Port GPIOC
#define G0_Pin GPIO_PIN_0
#define G0_GPIO_Port GPIOB
#define G1_Pin GPIO_PIN_1
#define G1_GPIO_Port GPIOB
#define G2_Pin GPIO_PIN_2
#define G2_GPIO_Port GPIOB
#define B6_Pin GPIO_PIN_6
#define B6_GPIO_Port GPIOC
#define B7_Pin GPIO_PIN_7
#define B7_GPIO_Port GPIOC
#define SYNC_PIXEL_Pin GPIO_PIN_8
#define SYNC_PIXEL_GPIO_Port GPIOC
#define SYNC_ROW_Pin GPIO_PIN_9
#define SYNC_ROW_GPIO_Port GPIOC
#define SYNC_FRAME_Pin GPIO_PIN_10
#define SYNC_FRAME_GPIO_Port GPIOC
#define SYNC_Pin GPIO_PIN_11
#define SYNC_GPIO_Port GPIOC
#define SYNC_EXTI_IRQn EXTI15_10_IRQn
#define G3_Pin GPIO_PIN_3
#define G3_GPIO_Port GPIOB
#define G4_Pin GPIO_PIN_4
#define G4_GPIO_Port GPIOB
#define G5_Pin GPIO_PIN_5
#define G5_GPIO_Port GPIOB
#define G6_Pin GPIO_PIN_6
#define G6_GPIO_Port GPIOB
#define G7_Pin GPIO_PIN_7
#define G7_GPIO_Port GPIOB
#define WR_Pin GPIO_PIN_8
#define WR_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_9
#define CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "frame.h"
#include "memory.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ITTER WIDTH*HIGH
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum _Status_Tim{
	Status_Tim_on,
	Status_Tim_off
} Status_Tim;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

__IO uint32_t row[200];
__IO uint32_t itter = 0;
void copy_to_buffers(const uint32_t* frame_buffer, uint32_t* buffer);
__IO int8_t data = 0;
__IO Status_Tim status = Status_Tim_off;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t sine_val[1000];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim10);

  //HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t*)frame, 200*120, DAC_ALIGN_12B_R);
  //copy_to_buffers((uint32_t*)&frame, (uint32_t*)&row);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		/*
		GPIOA->ODR = (GPIOA->ODR & 0xFF00) | (0x80); // for USB safety PA11 and PA12
		GPIOB->ODR = (GPIOB->ODR & 0xFF00) | (0x80);
		GPIOC->ODR = (GPIOC->ODR & 0xFA00) | (0x80);
    */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{

	}
}*/
void copy_to_buffers(const uint32_t* frame_buffer, uint32_t* buffer)
{
	memcpy((uint8_t*)buffer,(uint8_t*)frame_buffer + (itter*WIDTH), WIDTH * sizeof(uint32_t));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(SYNC_GPIO_Port, SYNC_Pin) == GPIO_PIN_SET){
		status = Status_Tim_off;
	}
	else{
		status = Status_Tim_on;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(status != Status_Tim_on)
	{
		return;
	}
	GPIOC->ODR ^= (1 << 8); // PC8 toggle it should be high
	GPIOA->ODR = (GPIOA->ODR & 0xFF00) | (0xff & (frame[itter]>>16)); // for USB safety PA11 and PA12
	GPIOB->ODR = (GPIOB->ODR & 0xFF00) | (0xff & (frame[itter]>>8));
	GPIOC->ODR = (GPIOC->ODR & 0xFF00) | (0xff & frame[itter]); // toggle PC8 and PC10 if they are high
	GPIOC->ODR ^= (1 << 8);
	//GPIOA->ODR = (GPIOA->ODR & 0xFF00) | (0xff & (data)); // for USB safety PA11 and PA12
	//GPIOB->ODR = 0xff & (data);
	//GPIOC->ODR = 0xff & data; // toggle PC8 and PC10 if they are high
	//data++;

	itter++;
	if(itter >= MAX_ITTER)
	{
		GPIOC->ODR ^= (1 << 10); // PC10 toggle it should be high
		itter= 0;
		status = Status_Tim_off;
		GPIOC->ODR ^= (1 << 10);
	}
}

/*void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	copy_to_buffers((uint8_t*)&rg_frame, (uint8_t*)&RG);
	copy_to_buffers((uint8_t*)&gb_frame, (uint8_t*)&GB);

	itter++;
	if(itter == HIGH)
	{
		itter = 0;
	}

	HAL_GPIO_TogglePin(TEST_2_GPIO_Port, TEST_2_Pin);

}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

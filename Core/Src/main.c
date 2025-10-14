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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <pid.h>
#include <encoder.h>
<<<<<<< HEAD
#include <stdbool.h>
=======

>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
#include <AUNO.h> 
#include <stdio.h>
#include <string.h>
#include "math.h"
#include <motor.h>
#include <JY61.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float yaw_offset = 0;
<<<<<<< HEAD
float NO1_yaw;
 
=======
float target_yaw;       // Ŀ��Ƕ�
float NO1_yaw;
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
<<<<<<< HEAD

=======
 
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

<<<<<<< HEAD
=======

>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
<<<<<<< HEAD
=======
  // debug: toggle PC13 to mark progress after MX_USART1_UART_Init
//  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	
HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxByte, 1);
    // debug: toggle PC13 to mark progress after starting UART receive
//    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
HAL_TIM_Encoder_Start (&htim2,TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start (&htim3,TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start (&htim4,TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start (&htim5,TIM_CHANNEL_ALL);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
HAL_TIM_Base_Start_IT(&htim1);
<<<<<<< HEAD
=======
static float target_yaw = 0;
  {
    // atomic read of stcAngle.Angle[2] using sequence counter
    uint8_t s1, s2;
    short angle_raw;
    do {
      s1 = stcAngle_seq;
      angle_raw = stcAngle.Angle[2];
      s2 = stcAngle_seq;
    } while (s1 != s2);
    current_yaw = ((float)angle_raw) / 32768.0f * 180.0f;
    target_yaw = current_yaw;
  }
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UART_ProcessBuffer();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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

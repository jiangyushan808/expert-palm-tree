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
#define Apower4_Pin GPIO_PIN_0
#define Apower4_GPIO_Port GPIOA
#define Bpower4_Pin GPIO_PIN_1
#define Bpower4_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_2
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_3
#define PWMB_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_6
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_7
#define BIN2_GPIO_Port GPIOA
#define BIN10_Pin GPIO_PIN_12
#define BIN10_GPIO_Port GPIOB
#define BIN20_Pin GPIO_PIN_13
#define BIN20_GPIO_Port GPIOB
#define AIN10_Pin GPIO_PIN_14
#define AIN10_GPIO_Port GPIOB
#define AIN20_Pin GPIO_PIN_15
#define AIN20_GPIO_Port GPIOB
#define PWMA0_Pin GPIO_PIN_8
#define PWMA0_GPIO_Port GPIOA
#define PWMB0_Pin GPIO_PIN_11
#define PWMB0_GPIO_Port GPIOA
#define Apower1_Pin GPIO_PIN_15
#define Apower1_GPIO_Port GPIOA
#define Bpower1_Pin GPIO_PIN_3
#define Bpower1_GPIO_Port GPIOB
#define Bpower2_Pin GPIO_PIN_4
#define Bpower2_GPIO_Port GPIOB
#define Apower2_Pin GPIO_PIN_5
#define Apower2_GPIO_Port GPIOB
#define Apower3_Pin GPIO_PIN_6
#define Apower3_GPIO_Port GPIOB
#define Bpower3_Pin GPIO_PIN_7
#define Bpower3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

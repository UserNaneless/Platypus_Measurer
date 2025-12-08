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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ADDR_1_Pin GPIO_PIN_14
#define ADDR_1_GPIO_Port GPIOC
#define ADDR_3_Pin GPIO_PIN_15
#define ADDR_3_GPIO_Port GPIOC
#define ADDR_2_Pin GPIO_PIN_0
#define ADDR_2_GPIO_Port GPIOA
#define ADDR_0_Pin GPIO_PIN_2
#define ADDR_0_GPIO_Port GPIOA
#define LED1_RGB_RED_Pin GPIO_PIN_0
#define LED1_RGB_RED_GPIO_Port GPIOB
#define LED1_RGB_GREEN_Pin GPIO_PIN_1
#define LED1_RGB_GREEN_GPIO_Port GPIOB
#define LED1_RGB_BLUE_Pin GPIO_PIN_12
#define LED1_RGB_BLUE_GPIO_Port GPIOB
#define B_ACCELEROMETER_Pin GPIO_PIN_14
#define B_ACCELEROMETER_GPIO_Port GPIOB
#define B_GYROSCOPE_Pin GPIO_PIN_15
#define B_GYROSCOPE_GPIO_Port GPIOB
#define ADDR_4_Pin GPIO_PIN_5
#define ADDR_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
void Switch_Effect(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN2_Pin GPIO_PIN_0
#define BTN2_GPIO_Port GPIOB
#define BTN2_EXTI_IRQn EXTI0_IRQn
#define BTN1_Pin GPIO_PIN_1
#define BTN1_GPIO_Port GPIOB
#define BTN1_EXTI_IRQn EXTI1_IRQn
#define LEDB_Pin GPIO_PIN_10
#define LEDB_GPIO_Port GPIOB
#define LEDG_Pin GPIO_PIN_11
#define LEDG_GPIO_Port GPIOB
#define LEDR_Pin GPIO_PIN_12
#define LEDR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TIME_LIMIT_1 5000 - 1
#define TIME_LIMIT_2 5000 - 1
#define TIME_CHANGE 2000 - 1
#define TIME_EFFECT_ADD_OR_SUB 1000 - 1
#define EFFECT_NUM_1 1
#define EFFECT_NUM_2 2
#define EFFECT_NUM_3 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

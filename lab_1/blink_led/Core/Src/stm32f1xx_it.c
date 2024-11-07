/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static int effect_num = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Switch_Effect(void)
{
	switch(effect_num)
	{
		case EFFECT_NUM_1:
		{
			effect_num = EFFECT_NUM_2;
			break;
		}
		case EFFECT_NUM_2:
		{
			HAL_GPIO_WritePin(GPIOB, LEDR_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LEDG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LEDB_Pin, GPIO_PIN_RESET);
			effect_num = EFFECT_NUM_3;
			break;
		}
		case EFFECT_NUM_3:
		{
			effect_num = EFFECT_NUM_1;
			break;
		}
	}
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(HAL_GPIO_ReadPin(GPIOB, BTN2_Pin) == 0)
	{
		HAL_TIM_Base_Start(&htim4);
	}
	else
	{
		uint32_t button_2_time = __HAL_TIM_GET_COUNTER(&htim4);
		uint32_t effect_time = __HAL_TIM_GET_COUNTER(&htim2);
		HAL_TIM_Base_Stop(&htim4);
		int time_greater_2 = (int)(button_2_time - TIME_LIMIT_2) / TIME_CHANGE;
		if(time_greater_2 < 0)
		{
			Switch_Effect();
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim2, effect_time - TIME_EFFECT_ADD_OR_SUB * (uint32_t)time_greater_2);
		}
	}

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BTN2_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(HAL_GPIO_ReadPin(GPIOB, BTN1_Pin) == 0)
	{
		HAL_TIM_Base_Start(&htim1);
	}
	else
	{
		uint32_t button_1_time = __HAL_TIM_GET_COUNTER(&htim1);
		uint32_t effect_time = __HAL_TIM_GET_COUNTER(&htim2);
		HAL_TIM_Base_Stop(&htim1);
		int time_greater_1 = (int)(button_1_time - TIME_LIMIT_1) / TIME_CHANGE;
		if(time_greater_1 < 0)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim2, effect_time + TIME_EFFECT_ADD_OR_SUB);
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim2, effect_time + TIME_EFFECT_ADD_OR_SUB * (uint32_t)time_greater_1);
		}
	}

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BTN1_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	switch(effect_num)
		{
			case EFFECT_NUM_1:
			{
				HAL_GPIO_WritePin(GPIOB, LEDR_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, LEDG_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, LEDB_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, LEDR_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LEDG_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LEDB_Pin, GPIO_PIN_RESET);
				break;
			}
			case EFFECT_NUM_2:
			{
				HAL_GPIO_TogglePin(GPIOB, LEDR_Pin);
				HAL_GPIO_TogglePin(GPIOB, LEDG_Pin);
				HAL_GPIO_TogglePin(GPIOB, LEDB_Pin);
				break;
			}
			case EFFECT_NUM_3:
			{
				HAL_GPIO_WritePin(GPIOB, LEDB_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, LEDG_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, LEDR_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, LEDR_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LEDG_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LEDB_Pin, GPIO_PIN_RESET);
				break;
			}
		}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

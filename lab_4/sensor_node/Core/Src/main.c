/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_Request_ID 0x712
#define CAN_Response_ID 0x7A2

#define ReadDataByLocalIdentifier_Request_SID 0x22
#define ReadDataByLocalIdentifier_Response_SID 0x62

#define WriteDataByLocalIdentifier_Request_SID 0x2E
#define WriteDataByLocalIdentifier_Response_SID 0x6E

#define SecurityAccess_Request_SID 0x27
#define SecurityAccess_Response_SID 0x67
#define Security_SEED_level 0x01
#define Security_KEY_level 0x02

#define RecordDataIdentifier_High_Byte 0x01
#define RecordDataIdentifier_Low_Byte 0x23

#define Identifier_Negative_Response 0x7F
#define Invalid_length_or_response_format 0x13
#define Invalid_minimum_request_length 0x13
#define DID_not_support 0x31
#define Invalid_Keys 0x35

#define Countinue_State 0x00
#define Wait_State 0x01

#define SF 0b0000
#define FF 0b0001
#define CF 0b0010
#define FC 0b0011
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader = {
		  .StdId = CAN_Request_ID,
		  .IDE = CAN_ID_STD,
		  .RTR = CAN_RTR_DATA,
		  .DLC = 8
  };
uint8_t TxData[8];
uint32_t TxMailbox;
CAN_FilterTypeDef Filter = {
		.FilterIdHigh = 0x0000,
		.FilterIdLow = 0x0000,
		.FilterMaskIdHigh = 0x0000,
		.FilterMaskIdLow = 0x0000,
		.FilterBank = 0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterFIFOAssignment = CAN_FILTER_FIFO1,
		.FilterActivation = CAN_FILTER_ENABLE
};
uint8_t SEED[4];
uint8_t KEY[16];
uint8_t SN = 0x0;
int MAX_KEY = 16;
int NEXT_KEY = 0;
int KEY_REMAIN = 0;
int Access_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Send_SF_Read(void)
{
	TxData[0] = (SF << 4) | 0x3;
	TxData[1] = ReadDataByLocalIdentifier_Request_SID;
	TxData[2] = RecordDataIdentifier_High_Byte;
	TxData[3] = RecordDataIdentifier_Low_Byte;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

void Read_Service_Handler(void)
{
	if(RxData[2] == RecordDataIdentifier_High_Byte && RxData[3] == RecordDataIdentifier_Low_Byte)
	{
		HAL_UART_Transmit(&huart1, RxData[4], 1, 100);
	}
}

void Send_SF_Write(void)
{
	if(Access_flag)
	{
		TxData[0] = (SF << 4) | 0x3;
		TxData[1] = WriteDataByLocalIdentifier_Request_SID;
		TxData[2] = RecordDataIdentifier_High_Byte;
		TxData[3] = RecordDataIdentifier_Low_Byte;
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	}
}

void Write_Service_Handler(void)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)"Write ok", 9, 100);
}

void Send_SF_Security(void)
{
	TxData[0] = (SF << 4) | 0x2;
	TxData[1] = SecurityAccess_Request_SID;
	TxData[2] = Security_SEED_level;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

void Send_FF_Security(void)
{
	TxData[0] = (FF << 4) | 0x0;
	TxData[1] = 0x06;
	TxData[2] = SecurityAccess_Request_SID;
	TxData[3] = Security_SEED_level;
	TxData[4] = KEY[0];
	TxData[5] = KEY[1];
	TxData[6] = KEY[2];
	TxData[7] = KEY[3];
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	NEXT_KEY = 4;
	KEY_REMAIN = MAX_KEY - NEXT_KEY;

}

void Send_CF_Security(void)
{
	TxData[0] = (CF << 4) | SN;
	for(int i = 1; i < KEY_REMAIN || i < 8; i++)
	{
		TxData[i] = KEY[NEXT_KEY];
		NEXT_KEY++;
	}
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	SN += 1;
	KEY_REMAIN = MAX_KEY - NEXT_KEY;
}

void Security_Handler(void)
{
	if(RxData[2] == Security_SEED_level)
	{
		Access_flag = 0;
		SEED[0] = RxData[3];
		SEED[1] = RxData[4];
		SEED[2] = RxData[5];
		SEED[3] = RxData[6];
		KEY[0] = SEED[0] ^ SEED[1];
		KEY[1] = SEED[1] + SEED[2];
		KEY[2] = SEED[2] ^ SEED[3];
		KEY[3] = SEED[3] + SEED[0];
		KEY[4] = SEED[0] | SEED[1];
		KEY[5] = SEED[1] + SEED[2];
		KEY[6] = SEED[2] | SEED[3];
		KEY[7] = SEED[3] + SEED[0];
		KEY[8] = SEED[0] & SEED[1];
		KEY[9] = SEED[1] + SEED[2];
		KEY[10] = SEED[2] & SEED[3];
		KEY[11] = SEED[3] + SEED[0];
		KEY[12] = SEED[0] - SEED[1];
		KEY[13] = SEED[1] + SEED[2];
		KEY[14] = SEED[2] - SEED[3];
		KEY[15] = SEED[3] + SEED[0];
	}
	else if(RxData[2] == Security_KEY_level)
	{
		Access_flag = 1;
		HAL_UART_Transmit(&huart1, (uint8_t*)"Key accept", 11, 100);
	}
}

void CAN_Error_Handler(void)
{
	char text[34];
	sprintf(text, "Ma loi 0x%u\n\rtu service 0x%u", RxData[3], RxData[2]);
	HAL_UART_Transmit(&huart1, (uint8_t*)text, 31, 100);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(KEY_REMAIN > 0)
	{
		Send_CF_Security();
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim2);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
		  {
			  if ((RxHeader.StdId == CAN_Response_ID))
			   {
				  switch(RxData[0] >> 4)
				  {
				  case SF:
					  switch(RxData[1])
					  {
					  case ReadDataByLocalIdentifier_Response_SID:
						  Read_Service_Handler();
						  break;
					  case SecurityAccess_Response_SID:
						  Security_Handler();
						  break;
					  case WriteDataByLocalIdentifier_Response_SID:
						  Write_Service_Handler();
						  break;
					  case Identifier_Negative_Response:
						  CAN_Error_Handler();
						  break;
					  }
					  break;
				  case FF:
					  break;
				  case CF:
					  break;
				  case FC:
					  if((RxData[0] & 0b00001111) == Countinue_State)
					  {
						  TxHeader.DLC = (int)RxData[1];
						  __HAL_TIM_SET_AUTORELOAD(&htim2, (int)RxData[2]);
						  HAL_TIM_Base_Start_IT(&htim2);
					  }
					  break;
				  }
			   }
		  }
}
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ConfigFilter(&hcan, &Filter);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  Send_SF_Read();
	  HAL_Delay(2000);
	  Send_SF_Security();
	  HAL_Delay(2000);
	  Send_SF_Write();
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "J1939.H"
#include "stdlib.h"
#include "J1939_Config.H"
#include <stdint.h>
#include <stdlib.h>
#include <switch.H>
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t response_data[8];
char uart_buf[50];
int uart_buf_len;
uint16_t timer_val;
uint16_t ran;
j1939_uint8_t  *info;
j1939_uint8_t _info;
struct Key_Info Key_Infovar;
struct Motorcontroller_Status Motorcontroller_Statusvar;
struct Switch_Info Switch_Infovar;
struct Motorcontroller_command Motorcontroller_commandvar;
struct BMS_Status BMS_Statusvar;
struct VCU_Status VCU_Statusvar;

uint8_t flag1=1;
uint8_t flag2=1;
uint8_t cnt1=0;
uint8_t  count=0;
uint8_t count1=0;
J1939_MESSAGE Message;
uint8_t vehicle_state=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendMsg_KEYINFO(J1939_MESSAGE *Message)
{

	J1939_MESSAGE _msgKEYINFO;
	_msgKEYINFO=*Message;

	//uint8_t buf[]="2";
	_msgKEYINFO.Mxe.DataPage = 0;
	_msgKEYINFO.Mxe.Priority = 0x01;
	_msgKEYINFO.Mxe.DestinationAddress = 0x33;//changed from 33 to 31//destination is all
	_msgKEYINFO.Mxe.DataLength = 8;
	_msgKEYINFO.Mxe.PDUFormat = 0xf1;
	_msgKEYINFO.Mxe.SourceAddress = 0x33;

	//	_msgKEYINFO.Mxe.Data[0] = 0;
	//	_msgKEYINFO.Mxe.Data[1] = 0;
	//	_msgKEYINFO.Mxe.Data[2] = 0;
	//	_msgKEYINFO.Mxe.Data[3] = 0;
	//	_msgKEYINFO.Mxe.Data[4] = 0;
	//	_msgKEYINFO.Mxe.Data[5] = 0;
	//	_msgKEYINFO.Mxe.Data[6] = 0;
	//	_msgKEYINFO.Mxe.Data[7] = 'k';

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message(&_msgKEYINFO)!= RC_SUCCESS);

	//HAL_UART_Transmit(&huart3,"send msg from keyinfo\r\n", 30, 1000);
}
void sendMsg_BMSSTATUS(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgBMSSTATUS;


	_msgBMSSTATUS=*Message;
	_msgBMSSTATUS.Mxe.DataPage = 0;
	_msgBMSSTATUS.Mxe.Priority = 0x03;
	_msgBMSSTATUS.Mxe.DestinationAddress = 0x22;
	_msgBMSSTATUS.Mxe.DataLength = 8;
	_msgBMSSTATUS.Mxe.PDUFormat = 0xF4;
	_msgBMSSTATUS.Mxe.SourceAddress = 0x44;

	//	_msgBMSSTATUS.Mxe.Data[0] = 0;
	//	_msgBMSSTATUS.Mxe.Data[1] = 0;
	//	_msgBMSSTATUS.Mxe.Data[2] = 0;
	//	_msgBMSSTATUS.Mxe.Data[3] = 0;
	//	_msgBMSSTATUS.Mxe.Data[4] = 0;
	//	_msgBMSSTATUS.Mxe.Data[5] = 0;
	//	_msgBMSSTATUS.Mxe.Data[6] = 0;
	//	_msgBMSSTATUS.Mxe.Data[7] = 0;

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgBMSSTATUS) != RC_SUCCESS);
}

void sendMsg_MOTORCONTROLLERSTATUS(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgMOTORCONTROLLERSTATUS;
	_msgMOTORCONTROLLERSTATUS =*Message;

	_msgMOTORCONTROLLERSTATUS.Mxe.DataPage = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Priority = 0x03;
	_msgMOTORCONTROLLERSTATUS.Mxe.DestinationAddress = 0x33;
	_msgMOTORCONTROLLERSTATUS.Mxe.DataLength = 1;
	_msgMOTORCONTROLLERSTATUS.Mxe.PDUFormat = 0xF2;
	_msgMOTORCONTROLLERSTATUS.Mxe.SourceAddress = 0x11;

	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[0] = Motorcontroller_Statusvar.motorcontroller_state;
	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[1] = 0;
	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[2] = 0;
	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[3] = 0;
	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[4] = 0;
	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[5] = 0;
	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[6] = 0;
	//	_msgMOTORCONTROLLERSTATUS.Mxe.Data[7] = 0;

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgMOTORCONTROLLERSTATUS) != RC_SUCCESS);


}

void sendMsg_VCUSTATUS(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgVCUSTATUS;
	_msgVCUSTATUS =*Message;
	_msgVCUSTATUS.Mxe.DataPage = 0;
	_msgVCUSTATUS.Mxe.Priority = 0x03;
	_msgVCUSTATUS.Mxe.DestinationAddress = 0x22;
	_msgVCUSTATUS.Mxe.DataLength = 8;
	_msgVCUSTATUS.Mxe.PDUFormat = 0x01;
	_msgVCUSTATUS.Mxe.SourceAddress = 0x22;

	//	_msgVCUSTATUS.Mxe.Data[0] = 0;
	//	_msgVCUSTATUS.Mxe.Data[1] = 0;
	//	_msgVCUSTATUS.Mxe.Data[2] = 0;
	//	_msgVCUSTATUS.Mxe.Data[3] = 0;
	//	_msgVCUSTATUS.Mxe.Data[4] = 0;
	//	_msgVCUSTATUS.Mxe.Data[5] = 0;
	//	_msgVCUSTATUS.Mxe.Data[6] = 0;
	//	_msgVCUSTATUS.Mxe.Data[7] = 0;

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message(&_msgVCUSTATUS) != RC_SUCCESS);


}

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
	Key_Infovar.Key_In=0;
	Key_Infovar.Key_Off=0;
	//	Microcontroller_Statusvar.motorcontroller_state=0;
	Motorcontroller_commandvar.brake=0;
	Motorcontroller_commandvar.ignition_command=0;

	Switch_Infovar.killswitch=0;
	Switch_Infovar.brake=0;
	Switch_Infovar.indicator=0;
	Switch_Infovar.highbeam=0;
	Switch_Infovar.navigation=0;
	BMS_Statusvar.BMS_Status=0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);//stb used only microchip
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);//voltage and same pin for navigatin_5
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,1);//HIGH BEAM
	//  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);//COMBO LIGHT
	//  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,1);//RIGHT INDICATOR
	//  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);//BRAKE LIGHT
	//  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);//LEFT INDICATOR
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,1);//STOP LIGHT
	// HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);//stb used only microchip
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);//voltage
	//HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR);// interrupts activation
	J1939_SetAddressFilter();
	if(HAL_CAN_Start(&hcan1)!=HAL_OK)
	{
		//		HAL_UART_Transmit(&huart3,"error in can_start\n\r", 20, 1000);
		Error_Handler();

	}
	J1939_Initialization();


	int a=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		switches_call();
		HAL_Delay(15);
		J1939_Poll( );

		while(vehicle_state)
		{
			switches_call();
			if(count1%2==0)
			{
				vehicle_state=0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  htim2.Init.Prescaler = 10000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_IN_Pin */
  GPIO_InitStruct.Pin = KEY_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(vehicle_state==1)
	{
		//	if(!vehicle_state)
		//		vehicle_state=1;
		//	else
		//		vehicle_state=0;

		if(count==count1)
		{
			Message.Mxe.Data[0]=_info1;


		}
		else
		{
			Message.Mxe.Data[0]=0x02;

		}
		sendMsg_KEYINFO(&Message);
		memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		//__NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
		//NVIC_ClearPendingIRQ (EXTI15_10_IRQn);
		//__HAL_TIM_DISABLE(&htim2);
		HAL_TIM_Base_Stop_IT(&htim2);
	}

}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

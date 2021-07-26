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
#define BCU
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
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
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


void sendMsg_MOTORCONTROLLERCOMMANDS(J1939_MESSAGE *Message)
{
	J1939_MESSAGE _msgMOTORCONTROLLER_COMMANDS;
	_msgMOTORCONTROLLER_COMMANDS=*Message;


	//uint8_t buf[]="2";
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DataPage = 0;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.Priority = 0x03;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DestinationAddress = 0x11;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DataLength = 8;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.PDUFormat = 0x17;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.SourceAddress = 0x33;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.PGN=0x1700;


	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[0] = 0x05;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[1] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[2] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[3] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[4] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[5] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[6] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[7] = '3';



	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgMOTORCONTROLLER_COMMANDS) != RC_SUCCESS);
//	HAL_UART_Transmit(&huart3,"send msg from switchmotor\r\n", 30, 1000);

}

void sendMsg_SWITCHINFO(J1939_MESSAGE *Message)
{

	J1939_MESSAGE _msgSWITCHINFO;
	_msgSWITCHINFO=*Message;

	//uint8_t buf[]="2";
	_msgSWITCHINFO.Mxe.DataPage = 0;
	_msgSWITCHINFO.Mxe.Priority = 0x03;
	_msgSWITCHINFO.Mxe.DestinationAddress = 0x22;
	_msgSWITCHINFO.Mxe.DataLength = 8;
	_msgSWITCHINFO.Mxe.PDUFormat = 0x15;
	_msgSWITCHINFO.Mxe.SourceAddress = 0x33;


	//	_msgSWITCHINFO.Mxe.Data[0] = 0;
	//	_msgSWITCHINFO.Mxe.Data[1] = 0;
	//	_msgSWITCHINFO.Mxe.Data[2] = 0;
	//	_msgSWITCHINFO.Mxe.Data[3] = 0;
	//	_msgSWITCHINFO.Mxe.Data[4] = 0;
	//	_msgSWITCHINFO.Mxe.Data[5] = 0;
	//	_msgSWITCHINFO.Mxe.Data[6] = 0;
	//	_msgSWITCHINFO.Mxe.Data[7] = 's';

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgSWITCHINFO) != RC_SUCCESS);
//	HAL_UART_Transmit(&huart3,"send msg from switchinfo\r\n", 30, 1000);

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
	_msgMOTORCONTROLLERSTATUS.Mxe.DestinationAddress = 0x33;//ds to all
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

void readMsg( )
{

	J1939_MESSAGE _msg;

	//		j1939_uint8_t _data[50];
	//		TP_RX_MESSAGE _tp_msg;
	//		uint8_t s;
	//		_tp_msg.data = _data;
	//		_tp_msg.data_num=50;
	//		if(J1939_TP_RX_Message(&_tp_msg)==RC_SUCCESS)
	//		    {
	//
	//		    	   s++;
	//		    }

	if(J1939_Read_Message(&_msg) == RC_SUCCESS)
	{
		//BMS STATUS
		if(_msg.Mxe.PGN == 0xF422)
		{
			uint8_t a;
			a++;

		}
		//MOTORCONTROLLER STATUS
		else if(_msg.Mxe.PGN == 0xF233)
		{
			flag1=0;


		}
		//VCU STATUS
		else if(_msg.Mxe.PGN == 0X100)
		{
			flag2=0;


		}
		else
			;
	}

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
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);//stb used only microchip
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);//voltage
	//HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR);// interrupts activation
	J1939_SetAddressFilter();
	if(HAL_CAN_Start(&hcan1)!=HAL_OK)
	{
//		HAL_UART_Transmit(&huart3,"error in can_start\n\r", 20, 1000);
		Error_Handler();

	}
	J1939_Initialization();
	//HAL_TIM_Base_Start(&htim6);
	//HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		switches_call();


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  htim2.Init.Prescaler = 1000-1;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|number_plate_Pin|GPIO_PIN_10
                          |high_beam_Pin|stoplight_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LED1_Pin|LED2_Pin|GPIO_PIN_14
                          |low_beam_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(brake_light_GPIO_Port, brake_light_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : AUX_Pin RIGHT_INDICATOR_Pin LEFT_INDICATOR_Pin navigation_4_Pin
                           PC8 ignition_Pin killl_switch_Pin PC11
                           PC12 */
  GPIO_InitStruct.Pin = AUX_Pin|RIGHT_INDICATOR_Pin|LEFT_INDICATOR_Pin|navigation_4_Pin
                          |GPIO_PIN_8|ignition_Pin|killl_switch_Pin|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MIDDLE_INDICATOR_Pin navigation_1_Pin PA7 */
  GPIO_InitStruct.Pin = MIDDLE_INDICATOR_Pin|navigation_1_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 number_plate_Pin PA10
                           high_beam_Pin stoplight_Pin PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|number_plate_Pin|GPIO_PIN_10
                          |high_beam_Pin|stoplight_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 LED1_Pin LED2_Pin low_beam_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LED1_Pin|LED2_Pin|low_beam_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : navigation_Pin brake_front_Pin */
  GPIO_InitStruct.Pin = navigation_Pin|brake_front_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : brake_light_Pin */
  GPIO_InitStruct.Pin = brake_light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(brake_light_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */





	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
	{
		if(GPIO_PIN==GPIO_PIN_11)
		{
			count++;//odd num on even off the key
			//count1=count;
			if(count%2==1)
			{
				count1=count;
				Key_Infovar.Key_Off=0;
				Key_Infovar.Key_In=1;
				Message.Mxe.Data[0]=0x02;
				HAL_TIM_Base_Start(&htim6);
				timer_val = __HAL_TIM_GET_COUNTER(&htim6);
				__HAL_TIM_ENABLE(&htim2);
				HAL_TIM_Base_Start_IT(&htim2);
				timer_val = __HAL_TIM_GET_COUNTER(&htim6) - timer_val;

				//sendMsg_KEYINFO(&Message);
				//memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

			}
			else
			{
				//send message to all nodes
				//count1=count;
				Key_Infovar.Key_Off=1;
				Key_Infovar.Key_In=0;
				Message.Mxe.Data[0]=0x01;
				sendMsg_KEYINFO(&Message);
				memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));
			}
			if(count>=254)
				count=0;

			//__NVIC_ClearPendingIRQ(EXTI1_IRQn);// to avoid key bouncing
		}
		else if(GPIO_PIN==GPIO_PIN_10)
		{
			//kill switch
			//when kill is pressed ignition should be off

			Switch_Infovar.killswitch=1;
			Message.Mxe.Data[0]=0x04;
			sendMsg_SWITCHINFO(&Message);
			memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		}
		else if(GPIO_PIN==GPIO_PIN_9)
		{
			//ignition switch
			Motorcontroller_commandvar.ignition_command=1;
			Message.Mxe.Data[0]=0x01;//
			sendMsg_MOTORCONTROLLERCOMMANDS(&Message);
			memset(Message.Mxe.Data, 0, 1*sizeof(Message.Mxe.Data[0]));

		}
		else if(GPIO_PIN==GPIO_PIN_4)
		{
			//navigation key_1
			uint8_t a;
			a++;
		}
		else if(GPIO_PIN==GPIO_PIN_7)
		{
			//navigation key_2
			uint8_t b;
			b++;


		}
		else if(GPIO_PIN==GPIO_PIN_15)
		{
			//navigation key_3
			uint8_t c;
			c++;


		}
		else if(GPIO_PIN==GPIO_PIN_6)
		{
			//navigation key_4
			uint8_t d;
			d++;

		}
		else if(GPIO_PIN==GPIO_PIN_8)
		{
			//navigation key_5
			uint8_t e;
			e++;


		}
		else if(GPIO_PIN==GPIO_PIN_2)
		{
			//right_indicator
			uint8_t f;
			f++;

		}
		else if(GPIO_PIN==GPIO_PIN_2)
		{
			//left_indicator
		}
		else if(GPIO_PIN==GPIO_PIN_8)
		{
			//brake_front
			uint8_t g;
			g++;

		}
		else
		{
			;
		}




	}


	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
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

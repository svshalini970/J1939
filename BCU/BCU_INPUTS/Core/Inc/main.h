/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AUX_Pin GPIO_PIN_13
#define AUX_GPIO_Port GPIOC
#define RIGHT_INDICATOR_Pin GPIO_PIN_2
#define RIGHT_INDICATOR_GPIO_Port GPIOC
#define LEFT_INDICATOR_Pin GPIO_PIN_3
#define LEFT_INDICATOR_GPIO_Port GPIOC
#define MIDDLE_INDICATOR_Pin GPIO_PIN_0
#define MIDDLE_INDICATOR_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_5
#define PWM_GPIO_Port GPIOA
#define MOBILE_CHARGER_Pin GPIO_PIN_6
#define MOBILE_CHARGER_GPIO_Port GPIOA
#define STORAGE_LIGHT_Pin GPIO_PIN_0
#define STORAGE_LIGHT_GPIO_Port GPIOB
#define BRAKE_LIGHT_Pin GPIO_PIN_7
#define BRAKE_LIGHT_GPIO_Port GPIOC
#define IGNITION_Pin GPIO_PIN_9
#define IGNITION_GPIO_Port GPIOC
#define NUMBER_PLATE_Pin GPIO_PIN_8
#define NUMBER_PLATE_GPIO_Port GPIOA
#define HIGH_BEAM_Pin GPIO_PIN_11
#define HIGH_BEAM_GPIO_Port GPIOA
#define R_INDI_LIGHT_Pin GPIO_PIN_15
#define R_INDI_LIGHT_GPIO_Port GPIOA
#define KILL_SWITCH_Pin GPIO_PIN_10
#define KILL_SWITCH_GPIO_Port GPIOC
#define KEY_IN_Pin GPIO_PIN_11
#define KEY_IN_GPIO_Port GPIOC
#define HIGH_BEAMC12_Pin GPIO_PIN_12
#define HIGH_BEAMC12_GPIO_Port GPIOC
#define L_INDI_LIGHT_Pin GPIO_PIN_2
#define L_INDI_LIGHT_GPIO_Port GPIOD
#define _OP_Pin GPIO_PIN_5
#define _OP_GPIO_Port GPIOB
#define BRAKE_FRONT_Pin GPIO_PIN_8
#define BRAKE_FRONT_GPIO_Port GPIOB
#define BRAKE_COMBO_Pin GPIO_PIN_9
#define BRAKE_COMBO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

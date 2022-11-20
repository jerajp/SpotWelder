/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"

#include "stm32f1xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern uint32_t PulseTime_ms;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define BUTTONTHRESHOLD 10
#define DEF_PULSE_LENGTH 10 //[ms]
#define MAX_PULSE_LENGTH 500 //[ms]
#define MIN_PULSE_LENGTH 1 //[ms]
#define PULSE_LED_TIME 3000 //[ms]

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint32_t watch1,watch2,watch3,watch4;

#define ADC_TO_mV (3.2) // Resistors (R1=3900 R2=2x680)
#define BANK_MAX_mV (7000)

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define START_LED_Pin GPIO_PIN_14
#define START_LED_GPIO_Port GPIOC
#define BANK_VOLTAGE_Pin GPIO_PIN_0
#define BANK_VOLTAGE_GPIO_Port GPIOA
#define RELAY_DISCHARGE_Pin GPIO_PIN_10
#define RELAY_DISCHARGE_GPIO_Port GPIOB
#define RELAY_CHARGE_Pin GPIO_PIN_11
#define RELAY_CHARGE_GPIO_Port GPIOB
#define PULSE_Pin GPIO_PIN_8
#define PULSE_GPIO_Port GPIOA
#define BUTTON1_Pin GPIO_PIN_4
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_5
#define BUTTON2_GPIO_Port GPIOB
#define FOOTSW_Pin GPIO_PIN_6
#define FOOTSW_GPIO_Port GPIOB
#define BUTTON3_Pin GPIO_PIN_7
#define BUTTON3_GPIO_Port GPIOB
#define LED_CLK_Pin GPIO_PIN_8
#define LED_CLK_GPIO_Port GPIOB
#define LED_DIO_Pin GPIO_PIN_9
#define LED_DIO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

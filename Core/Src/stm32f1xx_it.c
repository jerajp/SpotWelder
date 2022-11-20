/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
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

uint32_t TPlusStatus,TMinusStatus,Footsw_Status,TChargeStatus;
uint32_t TPlusStatusDebounce,TMinusStatusDebounce,FootswStatus_Debounce,TChargeStatusDebounce;
uint32_t TPlusStatusDebounceHIST,TMinusStatusDebounceHIST,FootswStatus_DebounceHIST,TChargeStatusDebounceHIST;
uint32_t TPlusCount,TMinusCount,FootswCount,TChargeCount;

uint32_t PulseActive;
uint32_t PulseActiveCnt;

uint32_t ChargeControl;
uint32_t Bank_mV;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

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

  //Bank voltage---------------------------------------------------------------------
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,10);
  Bank_mV=HAL_ADC_GetValue(&hadc1)*ADC_TO_mV;

  if(Bank_mV > BANK_MAX_mV)
  {
	  ChargeControl=0;

	  HAL_GPIO_WritePin(RELAY_CHARGE_GPIO_Port, RELAY_CHARGE_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(RELAY_DISCHARGE_GPIO_Port, RELAY_DISCHARGE_Pin,GPIO_PIN_SET);
  }
  //----------------------------------------------------------------------------------

  //Get Current Values
  TPlusStatus=!HAL_GPIO_ReadPin(BUTTON1_GPIO_Port,BUTTON1_Pin);
  TMinusStatus=!HAL_GPIO_ReadPin(BUTTON2_GPIO_Port,BUTTON2_Pin);
  TChargeStatus=!HAL_GPIO_ReadPin(BUTTON3_GPIO_Port,BUTTON3_Pin);
  Footsw_Status=!HAL_GPIO_ReadPin(FOOTSW_GPIO_Port,FOOTSW_Pin);

  //Save Values from previus loop
  TPlusStatusDebounceHIST=TPlusStatusDebounce;
  TMinusStatusDebounceHIST=TMinusStatusDebounce;
  TChargeStatusDebounceHIST=TChargeStatusDebounce;
  FootswStatus_DebounceHIST=FootswStatus_Debounce;

  // Debounce Button ++ Button -------------
  if(TPlusStatus==1)
  {
	  TPlusCount++;
  }
  else
  {
	  TPlusCount=0;
	  TPlusStatusDebounce=0;
  }
  if(TPlusCount==BUTTONTHRESHOLD)
  {
	  TPlusStatusDebounce=1;
  }
  //-----------------------------------------

  // Debounce -- Button ---------------------
  if(TMinusStatus==1)
  {
	  TMinusCount++;
  }
  else
  {
	  TMinusCount=0;
	  TMinusStatusDebounce=0;
  }
  if(TMinusCount==BUTTONTHRESHOLD)
  {
	  TMinusStatusDebounce=1;
  }
  //-----------------------------------------

  // Debounce Charge Button ---------------------
  if(TChargeStatus==1)
  {
	  TChargeCount++;
  }
  else
  {
	  TChargeCount=0;
	  TChargeStatusDebounce=0;
  }
  if(TChargeCount==BUTTONTHRESHOLD)
  {
	  TChargeStatusDebounce=1;
  }
  //-----------------------------------------

  // Debounce Footswitch --------------------
  if(Footsw_Status==1)
  {
	  FootswCount++;
  }
  else
  {
	  FootswCount=0;
	  FootswStatus_Debounce=0;
  }
  if(FootswCount==BUTTONTHRESHOLD)
  {
	  FootswStatus_Debounce=1;
  }
  //------------------------------------------

  // Charge Button Event
  if(TChargeStatusDebounceHIST!=TChargeStatusDebounce && TChargeStatusDebounce==1)
  {
	  if(Bank_mV > BANK_MAX_mV)ChargeControl=0;
	  else ChargeControl=!ChargeControl;

	  if(ChargeControl)
	  {
		  HAL_GPIO_WritePin(RELAY_CHARGE_GPIO_Port, RELAY_CHARGE_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(RELAY_DISCHARGE_GPIO_Port, RELAY_DISCHARGE_Pin,GPIO_PIN_RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(RELAY_CHARGE_GPIO_Port, RELAY_CHARGE_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(RELAY_DISCHARGE_GPIO_Port, RELAY_DISCHARGE_Pin,GPIO_PIN_SET);
	  }
  }

  //++ Button Event
  if(TPlusStatusDebounceHIST!=TPlusStatusDebounce && TPlusStatusDebounce==1)
  {
	  PulseTime_ms++;
	  if(PulseTime_ms>=MAX_PULSE_LENGTH)PulseTime_ms=MAX_PULSE_LENGTH;
	  LL_TIM_OC_SetCompareCH1(TIM1, PulseTime_ms*100);
  }

  //-- Button Event
  if(TMinusStatusDebounceHIST!=TMinusStatusDebounce && TMinusStatusDebounce==1)
  {
	  PulseTime_ms--;
	  if(PulseTime_ms<=MIN_PULSE_LENGTH)PulseTime_ms=MIN_PULSE_LENGTH;
	  LL_TIM_OC_SetCompareCH1(TIM1, PulseTime_ms*100);
  }

  //Footswitch Event
  if(FootswStatus_DebounceHIST!=FootswStatus_Debounce && FootswStatus_Debounce==1)
  {
	  //Check if Bank OK
	  //Start Pulse
	  HAL_GPIO_WritePin(START_LED_GPIO_Port, START_LED_Pin,GPIO_PIN_SET);
	  PulseActive=1;
  }

  //WHEN PULSE IS ACTIVE FLASH LED-------------------------------------
  if(PulseActive)PulseActiveCnt++;
  if(PulseActiveCnt >= PULSE_LED_TIME )
  {
	  PulseActive=0;
	  PulseActiveCnt=0;
	  HAL_GPIO_WritePin(START_LED_GPIO_Port, START_LED_Pin,GPIO_PIN_RESET);

	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	  LL_TIM_EnableCounter(TIM1);
	  //Timer is in One pulse mode, Counter at Period goes to 0 and freezes
	  //With output enabled in PWM1 mode, pulse is always ON
	  //One solution is to disable output with IT on compare1
	  //PWM2 mode is more direct but introduces almost whole period start delay (0.65sec) which is worse
  }
  //--------------------------------------------------------------------


  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
  LL_TIM_ClearFlag_CC1(TIM1);
  LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);

  /* USER CODE END TIM1_CC_IRQn 0 */
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

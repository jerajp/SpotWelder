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

uint32_t TPlusStatus,TMinusStatus,Footsw_Status;
uint32_t TPlusStatusDebounce,TMinusStatusDebounce,FootswStatus_Debounce;
uint32_t TPlusStatusDebounceHIST,TMinusStatusDebounceHIST,FootswStatus_DebounceHIST;
uint32_t TPlusCount,TMinusCount,FootswCount;

uint32_t PulseActive;
uint32_t PulseActiveCnt;
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

  //Get Current Values
  TPlusStatus=!HAL_GPIO_ReadPin(BUTTON1_GPIO_Port,BUTTON1_Pin);
  TMinusStatus=!HAL_GPIO_ReadPin(BUTTON2_GPIO_Port,BUTTON2_Pin);
  Footsw_Status=!HAL_GPIO_ReadPin(FOOTSW_GPIO_Port,FOOTSW_Pin);

  //Save Values from previus loop
  TPlusStatusDebounceHIST=TPlusStatusDebounce;
  TMinusStatusDebounceHIST=TMinusStatusDebounce;
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

  //++ Button Event
  if(TPlusStatusDebounceHIST!=TPlusStatusDebounce && TPlusStatusDebounce==1)
  {
	  PulseTime_ms++;
	  if(PulseTime_ms>=MAX_PULSE_LENGTH)PulseTime_ms=MAX_PULSE_LENGTH;
  }

  //-- Button Event
  if(TMinusStatusDebounceHIST!=TMinusStatusDebounce && TMinusStatusDebounce==1)
  {
	  PulseTime_ms--;
	  if(PulseTime_ms<=MIN_PULSE_LENGTH)PulseTime_ms=MIN_PULSE_LENGTH;
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

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

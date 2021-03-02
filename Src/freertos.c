/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include <stdlib.h>

#include "gpio.h"
#include "usart.h"
#include "adc.h"
//#include "iwdg.h"
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
/* USER CODE BEGIN Variables */
uint8_t adcReady;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for joystickTask */
osThreadId_t joystickTaskHandle;
const osThreadAttr_t joystickTask_attributes = {
  .name = "joystickTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for steerTask */
osThreadId_t steerTaskHandle;
const osThreadAttr_t steerTask_attributes = {
  .name = "steerTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for qMotor */
osMessageQueueId_t qMotorHandle;
const osMessageQueueAttr_t qMotor_attributes = {
  .name = "qMotor"
};
/* Definitions for qSteer */
osMessageQueueId_t qSteerHandle;
const osMessageQueueAttr_t qSteer_attributes = {
  .name = "qSteer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartJoistickTask(void *argument);
void StartMotorTask(void *argument);
void StartSteerTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of qMotor */
  qMotorHandle = osMessageQueueNew (64, sizeof(int16_t), &qMotor_attributes);

  /* creation of qSteer */
  qSteerHandle = osMessageQueueNew (64, sizeof(int16_t), &qSteer_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of joystickTask */
  joystickTaskHandle = osThreadNew(StartJoistickTask, NULL, &joystickTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

  /* creation of steerTask */
  steerTaskHandle = osThreadNew(StartSteerTask, NULL, &steerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  xprintf(&huart1, "Starting the blinker..\n");
  /* Infinite loop */
  for(;;)
  {
    ledOn();
    osDelay(LED_BLINK_PERIOD_MS / portTICK_PERIOD_MS);

    ledToggle();
    osDelay(LED_BLINK_PERIOD_MS / portTICK_PERIOD_MS);

    ledToggle();
    osDelay(LED_BLINK_PERIOD_MS / portTICK_PERIOD_MS);

    ledToggle();
    osDelay((1000 - 3 * LED_BLINK_PERIOD_MS) / portTICK_PERIOD_MS);

    //HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartJoistickTask */
/**
* @brief Function implementing the joystickTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJoistickTask */
void StartJoistickTask(void *argument)
{
  /* USER CODE BEGIN StartJoistickTask */
  volatile uint16_t adc[2 * JS_AVG_FACTOR] = { 0, };
  uint16_t posSteer = 0, posMotor = 0;

  adcReady = 0;

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  osDelay(50 / portTICK_PERIOD_MS);

  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&adc, JS_AVG_FACTOR);

  /* Infinite loop */
  for(;;)
  {
    uint32_t adcSteer = 0, adcMotor = 0;

    if (adcReady) {
      adcReady = 0;

      for (size_t i = 0; i < 2 * JS_AVG_FACTOR; ++i) {
        if (i % 2)
          adcSteer += adc[i];
        else
          adcMotor += adc[i];
      }

      adcSteer = ADC_MAX_VALUE - (adcSteer / JS_AVG_FACTOR);
      adcMotor = ADC_MAX_VALUE - (adcMotor / JS_AVG_FACTOR);

      if (abs(posSteer - adcSteer) > JS_TOLERANCE) {
        posSteer = adcSteer;
        xprintf(&huart1, "Steer: %4d\n", posSteer);
      }
      if (abs(posMotor - adcMotor) > JS_TOLERANCE) {
        posMotor = adcMotor;
        xprintf(&huart1, "Motor: %4d\n", posMotor);
      }



      //xprintf(&huart1, "Steer: %4d Motor: %4d\n", adcSteer, adcMotor);

      HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&adc, JS_AVG_FACTOR);
    }

    osDelay(50 / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartJoistickTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartSteerTask */
/**
* @brief Function implementing the steerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSteerTask */
void StartSteerTask(void *argument)
{
  /* USER CODE BEGIN StartSteerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSteerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

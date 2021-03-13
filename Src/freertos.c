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
#include "tim.h"
#include "queue.h"
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
  int16_t  posSteer = 0, posMotor = 0;
  int16_t   calSteer = 0, calMotor = 0, calCycle = 0;

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

      if (calCycle > JS_IDLE_CYCLES) {

        for (size_t i = 0; i < 2 * JS_AVG_FACTOR; ++i) {
          if (i % 2)
            adcSteer += adc[i];
          else
            adcMotor += adc[i];
        }

        if ((!calMotor) || (calCycle > JS_IDLE_CYCLES + 250))
          calMotor = adcMotor / JS_AVG_FACTOR;

        if ((!calSteer) || (calCycle > JS_IDLE_CYCLES + 250))
          calSteer = adcSteer / JS_AVG_FACTOR;

        adcSteer = (ADC_MAX_VALUE - (adcSteer / JS_AVG_FACTOR)) /*- calSteer*/;
        adcMotor = (ADC_MAX_VALUE - (adcMotor / JS_AVG_FACTOR)) /*- calMotor*/;

        if (abs(posSteer - adcSteer) > JS_TOLERANCE) {
          posSteer = adcSteer;
          if (uxQueueSpacesAvailable(qSteerHandle)) {
            xQueueSend(qSteerHandle, &posSteer, portMAX_DELAY);
          }

          //xprintf(&huart1, "Steer: %4d\n", posSteer);
        }
        if (abs(posMotor - adcMotor) > JS_TOLERANCE) {
          posMotor = adcMotor;

          if (uxQueueSpacesAvailable(qMotorHandle)) {
            xQueueSend(qMotorHandle, &posMotor, portMAX_DELAY);
          }

          //xprintf(&huart1, "Motor: %4d\n", posMotor);
        }

        /*if ((posSteer == adcSteer) && (posMotor == adcMotor) &&
            (posSteer < 100) && (posMotor < 100))
          calCycle++;
        else
          calCycle = JS_IDLE_CYCLES + 1;
        */

        //xprintf(&huart1, "Motor: %5d; Steer: %5d\n", posMotor, posSteer);

      }
      else {
        calCycle++;
      }
      HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&adc, JS_AVG_FACTOR);
    }

    osDelay(FREERTOS_TASK_PERIOD_MS / portTICK_PERIOD_MS);
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
  uint32_t arr = TIM4->ARR + 1;
  uint16_t code;
  int16_t throttle = 0;

  xprintf(&huart1, "Starting Motor PWM...\n");

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* Infinite loop */
  for(;;)
  {
    if (uxQueueMessagesWaiting(qMotorHandle)) {
      if (pdPASS == xQueueReceive(qMotorHandle, &code, portMAX_DELAY)) {
        // Forward slow decay
        if (code >= ((ADC_MAX_VALUE / 2) + JS_TOLERANCE * 2)) {

          throttle = map(code, ADC_MAX_VALUE / 2, ADC_MAX_VALUE, arr / 3, arr);
          throttle = constrain(throttle, 0, arr);

          TIM4->CCR3 = 0;
          TIM4->CCR4 = throttle;
          HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin, GPIO_PIN_SET);
        }
        // Reverse slow decay
        else if (code < ((ADC_MAX_VALUE / 2) - JS_TOLERANCE * 2)) {
          throttle = map(code, 0, ADC_MAX_VALUE / 2, arr, arr / 3);
          throttle = constrain(throttle, 0, arr);

          TIM4->CCR3 = throttle;
          TIM4->CCR4 = 0;
          HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin, GPIO_PIN_SET);
        }
        //Stop
        else {
          throttle = 0;

          TIM4->CCR3 = 0;
          TIM4->CCR4 = 0;
          HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin, GPIO_PIN_RESET);
        }
        //xprintf(&huart1, "Motor code: %4d; throttle: %4d\n", code, throttle);
      }
    }

    osDelay(FREERTOS_TASK_PERIOD_MS / portTICK_PERIOD_MS);
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
  uint32_t arr = TIM3->ARR + 1;
  uint16_t code, pwmCode, correction = -10000;
  uint8_t angle;

  xprintf(&huart1, "Starting Steering PWM...\n");

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  TIM3->CCR1 = arr / 20;
  /* Infinite loop */
  for(;;)
  {
    if (uxQueueMessagesWaiting(qSteerHandle)) {
      if (pdPASS == xQueueReceive(qSteerHandle, &code, portMAX_DELAY)) {

        angle = map(code,
            0, ADC_MAX_VALUE,
            SERVO_LIMIT_DEG + SERVO_CORRECTION, 180 - SERVO_LIMIT_DEG + SERVO_CORRECTION);
        angle = constrain(angle, SERVO_LIMIT_DEG, 180 - SERVO_LIMIT_DEG);

        pwmCode = map(angle, 0, 180, arr / 50, arr / 9);
        pwmCode = constrain(pwmCode, arr / 50, arr / 9);

        TIM3->CCR1 = pwmCode;
      }
      //xprintf(&huart1, "Steer code: %4d; angle: %4d, PWM code: %4d\n", code, angle, pwmCode);
    }

    osDelay(FREERTOS_TASK_PERIOD_MS / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartSteerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

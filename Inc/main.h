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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define constrain(amt,low,high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
float map(float val, float I_Min, float I_Max, float O_Min, float O_Max);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SERVO_Pin GPIO_PIN_6
#define SERVO_GPIO_Port GPIOA
#define MOTOR_SLEEP_Pin GPIO_PIN_6
#define MOTOR_SLEEP_GPIO_Port GPIOB
#define JOY_BTN_Pin GPIO_PIN_7
#define JOY_BTN_GPIO_Port GPIOB
#define MOTOR_IN1_Pin GPIO_PIN_8
#define MOTOR_IN1_GPIO_Port GPIOB
#define MOTOR_IN2_Pin GPIO_PIN_9
#define MOTOR_IN2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define FREERTOS_TASK_PERIOD_MS 50

#define LED_BLINK_PERIOD_MS     100

#define JS_AVG_FACTOR           16
#define JS_TOLERANCE            15
#define JS_IDLE_CYCLES          5

#define SERVO_LIMIT_DEG         20
#define SERVO_CORRECTION        -18

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

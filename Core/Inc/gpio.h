/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* Blue Pill onboard LED is usually connected to PC13 and is active-low. */
#define STATUS_LED_Pin        GPIO_PIN_13
#define STATUS_LED_GPIO_Port  GPIOC

/* HC-05 STATE input. Assumed HIGH when Bluetooth SPP link is connected. */
#define BT_STATE_Pin          GPIO_PIN_12
#define BT_STATE_GPIO_Port    GPIOB

/* Button input with internal pull-up. Active LOW when pressed. */
#define BTN_THROTTLE_Pin      GPIO_PIN_13
#define BTN_THROTTLE_GPIO_Port GPIOB
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */


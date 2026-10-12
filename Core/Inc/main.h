/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define l_key_Pin GPIO_PIN_13
#define l_key_GPIO_Port GPIOC
#define ws2812_Pin GPIO_PIN_5
#define ws2812_GPIO_Port GPIOA
#define OUT3_A_Pin GPIO_PIN_6
#define OUT3_A_GPIO_Port GPIOA
#define OUT2_B_Pin GPIO_PIN_1
#define OUT2_B_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOE
#define OUT1_C_Pin GPIO_PIN_10
#define OUT1_C_GPIO_Port GPIOE
#define OUT0_START_Pin GPIO_PIN_11
#define OUT0_START_GPIO_Port GPIOE
#define IN3_A_Pin GPIO_PIN_15
#define IN3_A_GPIO_Port GPIOD
#define IN2_B_Pin GPIO_PIN_6
#define IN2_B_GPIO_Port GPIOC
#define IN0_TOUCH_Pin GPIO_PIN_4
#define IN0_TOUCH_GPIO_Port GPIOA
#define IN1_C_Pin GPIO_PIN_8
#define IN1_C_GPIO_Port GPIOC
#define OUT_jidianqi_Pin GPIO_PIN_9
#define OUT_jidianqi_GPIO_Port GPIOC
#define START_Pin GPIO_PIN_8
#define START_GPIO_Port GPIOA
#define OUT_MOS_Pin GPIO_PIN_12
#define OUT_MOS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USART1 调试 printf 开关：1=开(会占用USART1发送) 0=关(串口专用于Modbus) */
#define ENABLE_USART1_DBG_PRINTF   0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

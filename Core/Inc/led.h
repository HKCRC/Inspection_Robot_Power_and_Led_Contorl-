/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    led.h
  * @brief   LED 控制：LED1=限位/微动指示，LED2=上位机可控
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Exported defines */
/* LED 点亮极性：1=高电平亮 0=低电平亮(按板子接线改) */
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH  1u
#endif
/* USER CODE END Exported defines */

/* USER CODE BEGIN Exported functions */
void    led_init(void);              /* 初始化：两灯全灭 */
void    led_task(void);              /* 主循环刷新：LED1=限位状态 */
void    led2_set(uint8_t on);        /* 上位机控制 LED2：1=亮 0=灭 */
uint8_t led2_get(void);              /* 上位机控制 LED2 当前状态 */
/* USER CODE END Exported functions */

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */

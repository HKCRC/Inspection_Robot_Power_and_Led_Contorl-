/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    di.h
  * @brief   数字量输入(DI)：IN1/IN2/IN3/TOUCH/START 五路
  * @note    低电平有效：输入为低电平(闭合/按下)时视为触发。
  *          含软件防抖，返回稳定状态；供 Modbus 寄存器上报。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __DI_H
#define __DI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* 输入路数：IN1/IN2/IN3/TOUCH/START */
#define DI_IN_MAX   5u

/* USER CODE BEGIN Exported functions */
void     di_init(void);                    /* 初始化并读取一次 */
void     di_scan(void);                    /* 主循环周期调用：采样+防抖 */
uint8_t  di_get_input(uint8_t idx);        /* 单路输入：0=IN1 1=IN2 2=IN3 3=TOUCH 4=START，1=触发 */
uint16_t di_get_packed(void);              /* 打包位图：bit0=IN1..bit4=START，1=触发 */
/* 兼容旧接口：限位已取消(恒 0)；TOUCH 单路状态见 di_get_touch() */
uint8_t  di_get_limit_left(void);          /* 恒 0(限位取消) */
uint8_t  di_get_limit_right(void);         /* 恒 0(限位取消) */
uint8_t  di_get_limit(void);               /* 恒 0(限位取消) */
uint8_t  di_get_touch(void);               /* TOUCH(机器人盖章)：1=触发；原名 di_get_estop() */
/* USER CODE END Exported functions */

#ifdef __cplusplus
}
#endif

#endif /* __DI_H */

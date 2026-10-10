/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ctl.h
  * @brief   输出控制逻辑层：按键联动输出 + 电源(MOS)锁存状态机
  * @note    本模块是全系统唯一的输出策略点：
  *          ① 按键联动：IN1/IN2/IN3 按下 -> 对应联动输出高电平(电平跟随)
  *          ② 电源锁存：START 检测到低电平(按下) -> OUT_MOS 高(通电，按下即
  *             上电)；此后仅上位机线圈指令(写线圈0)可通电/断电，
  *             本地按键不能断电。
  *          ③ OUT_START 随电源同态锁存：START 检测到按下置高并保持（按键无
  *             自锁，松开不回落），随上位机断电指令一并复位。
  *          ④ 继电器(线圈1)：吸合立即执行；关闭指令延时15s再输出高电平
  *             断开触点（继电器驱动为低电平吸合）。
  *          数据源为 di.c 防抖后的稳定状态，输出经 relay.c 通道驱动。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __CTL_H
#define __CTL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Exported functions */
void    ctl_init(void);                 /* 初始化：上电默认 MOS 断电/继电器吸合 */
void    ctl_task(void);                 /* 主循环周期调用：刷新联动输出/电源锁存/继电器延时 */
void    ctl_set_power(uint8_t on);      /* 上位机写线圈0时调用：1=MOS通电 0=断电 */
uint8_t ctl_power_get(void);            /* 当前电源锁存状态：1=通电 0=断电 */
void    ctl_set_relay(uint8_t on);      /* 上位机写线圈1时调用：1=吸合(立即) 0=关闭(延时15s) */
uint8_t ctl_relay_get(void);            /* 继电器逻辑状态：1=吸合 0=关闭(含延时进行中) */
/* USER CODE END Exported functions */

#ifdef __cplusplus
}
#endif

#endif /* __CTL_H */

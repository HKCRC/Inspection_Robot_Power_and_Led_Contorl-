/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    relay.h
  * @brief   数字输出(DO)统一驱动 API 头文件
  * @note    本模块是所有数字输出引脚的唯一驱动层：只做"通道号->引脚"映射，
  *          不含任何业务策略(按键联动/电源锁存/Modbus 语义在 ctl.c / modbus_rtu.c)。
  *          新增输出只需：main.h 加引脚宏 + s_do_map 表加一行 + 扩展通道宏。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* 数字输出通道总数：继电器 + MOS 管 + 4 路按键联动输出，如需扩展修改此宏并补充映射表 */
#define RELAY_CH_MAX   6

/* 通道号定义(从 1 开始)：与 relay.c 的 s_do_map[] 顺序一一对应 */
#define RELAY_CH_JIDIANQI    1u   /* 继电器        OUT_jidianqi/PC9  上位机线圈1控制 */
#define RELAY_CH_MOS         2u   /* MOS管(主电源) OUT_MOS/PC12      上位机线圈0/START键控制 */
#define RELAY_CH_OUT1_BULE   3u   /* 按键联动输出1 IN1_BLUE -> OUT1_BULE/PE10 */
#define RELAY_CH_OUT2_WHITE  4u   /* 按键联动输出2 IN2_WHITE -> OUT2_WHITE/PB1 */
#define RELAY_CH_OUT3_GREEN  5u   /* 按键联动输出3 IN3_GREEN -> OUT3_GREEN/PA6 */
#define RELAY_CH_OUT_START   6u   /* 按键联动输出4 START -> OUT0_START/PE11 */

#define RELAY_OFF      0u
#define RELAY_ON       1u

/* USER CODE BEGIN Exported functions */
uint8_t  relay_set(uint8_t ch, uint8_t on);   /* 单路输出：返回 1 成功 0 通道非法 */
uint16_t relay_get_state(void);               /* 查询所有输出状态，bitN=第N+1路，1=输出高 */
/* USER CODE END Exported functions */

void relay_init(void);                        /* 初始化：上电默认全部输出低 */

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */

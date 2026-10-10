/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    modbus_rtu.h
  * @brief   Modbus RTU 从机(Slave) 头文件
  * @note    基于 USART1 DMA + IDLE 空闲中断实现变长帧接收，
  *          支持功能码 0x01(读线圈) / 0x05(写单线圈)，
  *          线圈地址映射到继电器控制 API。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Exported functions */
void    modbus_init(void);   /* 初始化并启动接收 */
uint8_t modbus_task(void);   /* 主循环轮询：返回 1=收到上位机关闭(断电)继电器指令，0=否则 */
/* WS2812 配置通过保持寄存器 0x0006~0x000C 写入(FC06/FC16)，写入后内部
 * 直接推送给灯效引擎(ws2812.h)即时生效，读取走 FC03/FC04 返回寄存器镜像 */
/* USER CODE END Exported functions */

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_RTU_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ws2812_port.h
  * @brief   WS2812 输出驱动抽象层（灯效引擎与硬件 IO 之间的解耦层）
  * @note    灯效引擎(ws2812.c)只依赖本头文件，不接触任何具体外设；
  *          更换输出 IO 时只需改 WS_PORT_USE 宏（或新增一个后端并挂到
  *          ws2812_port.c 的选择分支），引擎与 Modbus 层零改动。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __WS2812_PORT_H
#define __WS2812_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Exported defines */
/* ---- 输出后端选择（更换 IO 只改这里）--------------------------------------
 * WS_PORT_SPI  : SPI4  MOSI = PE14（DMA 发送，时序最稳，默认）
 * WS_PORT_GPIO : GPIO  软件 800Kbps = PA5（ws2812_Pin，NOP 延时需示波器微调）
 * WS_PORT_PWM  : TIM3_CH1 PWM+DMA = PB4
 * 注意：多后端硬件同时接线时只能启用一个，避免并发驱动损坏帧数据
 * ------------------------------------------------------------------------ */
#define WS_PORT_SPI   0u
#define WS_PORT_GPIO  1u
#define WS_PORT_PWM   2u

#ifndef WS_PORT_USE
#define WS_PORT_USE   WS_PORT_PWM
#endif

/* 灯带物理灯珠总数（决定发送缓冲大小与每次下发的帧长）；
 * Modbus 寄存器 0x0006 配置其中"点亮的灯珠数量"，其余保持熄灭 */
#define WS2812_LED_MAX           1000u

/* WS2812/WS2812B 颜色顺序固定为 GRB，MSB 先发 */
#define WS2812_COLOR_RED         0xFF0000u   /* 红 */
#define WS2812_COLOR_GREEN       0x00FF00u   /* 绿 */
#define WS2812_COLOR_BLUE        0x0000FFu   /* 蓝 */
#define WS2812_COLOR_YELLOW      0xFFFF00u   /* 黄（红+绿） */
#define WS2812_COLOR_CYAN        0x00FFFFu   /* 青（绿+蓝） */
#define WS2812_COLOR_PURPLE      0xFF00FFu   /* 紫（红+蓝） */
#define WS2812_COLOR_WHITE       0xFFFFFFu   /* 白（三色） */
#define WS2812_COLOR_OFF         0x000000u   /* 熄灭 */
/* USER CODE END Exported defines */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/**
  * @brief  初始化所选输出后端（引脚/定时器/DMA），空闲输出复位电平
  */
void ws2812_port_init(void);

/**
  * @brief  发送一帧颜色数据到灯带（阻塞至整帧 + 复位码元发完再返回）
  * @param  grb    颜色帧缓冲，GRB 字节序：grb[3*i]=G, grb[3*i+1]=R, grb[3*i+2]=B
  * @param  count  灯珠数量（<= WS2812_LED_MAX）；=0 时不发送
  * @note   阻塞时长约 count*30us + 复位 300us（200 颗约 6.3ms），期间 CPU 空等 DMA
  */
void ws2812_port_send(const uint8_t *grb, uint16_t count);

/**
  * @brief  微秒级忙等延时（DWT CYCCNT，端口层内部用，也可被其它模块复用）
  */
void ws2812_port_delay_us(uint32_t us);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_PORT_H */

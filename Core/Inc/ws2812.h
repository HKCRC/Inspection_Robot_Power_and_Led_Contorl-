/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ws2812.h
  * @brief   WS2812 灯效引擎：常灭/常亮/闪烁/流水/呼吸，配置由 Modbus 寄存器推送
  * @note    分层：modbus_rtu.c --(ws2812_set_cfg)--> ws2812.c --(ws2812_port_send)
  *          --> ws2812_port.c(驱动抽象)。引擎不接触任何具体外设。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __WS2812_H
#define __WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ws2812_port.h"

/* USER CODE BEGIN Exported defines */
/* 灯效模式（对应 Modbus 寄存器 0x000B 的合法取值） */
#define WS_MODE_OFF      0u   /* 常灭 */
#define WS_MODE_SOLID    1u   /* 常亮 */
#define WS_MODE_BLINK    2u   /* 闪烁 1Hz（亮500ms/灭500ms） */
#define WS_MODE_FLOW     3u   /* 流水灯（speed=每步间隔 ms） */
#define WS_MODE_BREATH   4u   /* 呼吸灯（speed=全周期 ms，0->最亮->0） */
#define WS_MODE_MAX      WS_MODE_BREATH

/* 速度参数默认值（寄存器 0x000C，按模式解释） */
#define WS_SPEED_FLOW_DEF     50u     /* 流水：每步 50ms（160 灯一个来回约 8s） */
#define WS_SPEED_BREATH_DEF   2000u   /* 呼吸：全周期 2s */
/* USER CODE END Exported defines */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**
  * @brief  灯带配置（Modbus 寄存器 0x0006~0x000C 的解析结果）
  */
typedef struct
{
  uint16_t count;       /* 灯珠数量 0~WS2812_LED_MAX */
  uint8_t  mode;        /* WS_MODE_x */
  uint8_t  brightness;  /* 亮度 0~100 (%) */
  uint32_t color;       /* 颜色 RGB888，如 WS2812_COLOR_RED */
  uint16_t speed;       /* 速度：流水=步进ms(10~1000) 呼吸=周期ms(500~10000)，其它模式忽略 */
} ws_cfg_t;
/* USER CODE END ET */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void ws2812_init(void);                        /* 初始化引擎 + 输出后端，按默认配置渲染首帧 */
void ws2812_task(void);                        /* 主循环周期调用：驱动动效（内部非阻塞） */
void ws2812_set_cfg(const ws_cfg_t *cfg);      /* Modbus 写寄存器后推送新配置，立即生效 */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H */

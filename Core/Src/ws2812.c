/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ws2812.c
  * @brief   WS2812 灯效引擎实现
  * @note    非阻塞设计：基于 HAL_GetTick() 时间片驱动动效；仅发送帧时阻塞
  *          （约 6ms/帧 @160灯）。静态模式（常灭/常亮）只在配置变化时渲染一次，
  *          动态模式（闪烁/流水/呼吸）按帧节拍重绘，不空耗 CPU 与总线。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include <string.h>

/* USER CODE BEGIN Private defines */
#define WS_BLINK_HALF_MS   500u   /* 1Hz 闪烁：半周期 500ms（亮500ms/灭500ms） */
#define WS_FRAME_MS        20u    /* 动效重绘帧节拍 ~50fps */
#define WS_FLOW_TAIL       8u     /* 流水灯拖尾长度（颗） */

/* 速度参数钳位范围（Modbus 只做上限校验，细范围由引擎钳位兜底） */
#define WS_SPEED_FLOW_MIN     10u
#define WS_SPEED_FLOW_MAX     1000u
#define WS_SPEED_BREATH_MIN   500u
#define WS_SPEED_BREATH_MAX   10000u
/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
static uint16_t s_prev_count;   /* 上次渲染的灯珠数：灯数调小时清尾部 + 决定本帧下发长度 */
/* 上电默认配置：160 灯 / 红色 / 100% 亮度 / 常亮（与 Modbus 寄存器默认值一致） */
 ws_cfg_t s_cfg = {
    .count      = 400u,
    .mode       = WS_MODE_SOLID,
    .brightness = 100u,
    .color      = WS2812_COLOR_BLUE,
    .speed      = WS_SPEED_FLOW_DEF,
};

static uint8_t  s_frame[WS2812_LED_MAX * 3u];  /* GRB 帧缓冲，整帧下发给端口层 */
static uint8_t  s_need_render = 1u;            /* 配置变化后需立即渲染 */
static uint8_t  s_blink_on;                    /* 闪烁当前相位：1=亮 0=灭 */
static uint16_t s_flow_pos;                    /* 流水灯头部位置 */
static uint32_t s_t_frame;                     /* 上次动效帧 tick */
/* USER CODE END Private variables */

/* USER CODE BEGIN Private_Function_Prototypes */
static uint16_t ws_frame_len(void);            /* 本帧需要下发的灯珠数 */
static void     ws_flush(void);                /* 按需长度下发当前帧 */
static uint32_t scale_color(uint32_t color, uint8_t pct);
static void     put_pixel(uint16_t idx, uint32_t rgb);
static void     render_uniform(uint8_t pct);   /* 整条同色按 pct 再缩放亮度 */
static void     render_flow(void);
/* USER CODE END Private_Function_Prototypes */

/* USER CODE BEGIN Private_Functions */
/**
  * @brief  颜色按百分比 0~100 等比缩放（RGB888 各分量独立）
  */
static uint32_t scale_color(uint32_t color, uint8_t pct)
{
  uint32_t r = ((color >> 16) & 0xFFu) * pct / 100u;
  uint32_t g = ((color >> 8)  & 0xFFu) * pct / 100u;
  uint32_t b = ( color        & 0xFFu) * pct / 100u;
  return (r << 16) | (g << 8) | b;
}

/**
  * @brief  写一颗灯的 GRB 字节到帧缓冲（WS2812 颜色顺序 GRB）
  */
static void put_pixel(uint16_t idx, uint32_t rgb)
{
  s_frame[idx * 3u + 0u] = (uint8_t)((rgb >> 8)  & 0xFFu);   /* G */
  s_frame[idx * 3u + 1u] = (uint8_t)((rgb >> 16) & 0xFFu);   /* R */
  s_frame[idx * 3u + 2u] = (uint8_t)( rgb        & 0xFFu);   /* B */
}

/**
  * @brief  本帧需要下发的灯珠数 = max(当前灯数, 上次灯数)
  * @note   调小 0x0006(灯数) 时必须覆盖到旧灯数，才能把尾部残留灯珠刷成熄灭。
  */
static uint16_t ws_frame_len(void)
{
  uint16_t len = (s_cfg.count > s_prev_count) ? s_cfg.count : s_prev_count;

  if (len > WS2812_LED_MAX)
  {
    len = WS2812_LED_MAX;
  }
  return len;
}

/**
  * @brief  下发当前帧(只发 ws_frame_len() 颗)并记录本次灯数
  * @note   原实现固定下发 WS2812_LED_MAX(1000) 颗，PWM 后端一帧约 30ms，
  *         会把主循环拖慢并严重影响 Modbus 收发；这里按实际需要长度下发。
  */
static void ws_flush(void)
{
  uint16_t len = ws_frame_len();

  if (len == 0u)
  {
    return;                      /* 灯数为 0：无灯珠需要下发 */
  }
  ws2812_port_send(s_frame, len);
  s_prev_count = s_cfg.count;
}

/**
  * @brief  整条按配置颜色渲染，pct 为相对配置亮度的再缩放系数(0~100)
  */
static void render_uniform(uint8_t pct)
{
  uint32_t rgb = scale_color(s_cfg.color, (uint8_t)((s_cfg.brightness * pct) / 100u));
  for (uint16_t i = 0u; i < s_cfg.count; i++)
  {
    put_pixel(i, rgb);
  }
  /* 点亮数量之外的灯珠显式熄灭：只覆盖到"上次灯数"范围，
   * 避免调小 0x0006(灯数) 时尾部灯珠残留上一帧的旧颜色 */
  {
    uint16_t len = ws_frame_len();

    for (uint16_t i = s_cfg.count; i < len; i++)
    {
      put_pixel(i, WS2812_COLOR_OFF);
    }
  }
  ws_flush();   /* 只下发实际需要的长度，不再固定 1000 颗(30ms 阻塞) */
}

/**
  * @brief  渲染一帧流水灯：头部全亮，尾部线性衰减
  */
static void render_flow(void)
{
  uint16_t len = ws_frame_len();   /* 只处理/下发实际需要的长度 */

  memset(s_frame, 0, (uint32_t)len * 3u);

  for (uint16_t t = 0u; (t < WS_FLOW_TAIL) && (t < s_cfg.count); t++)
  {
    /* 第 t 颗在头部后方 t 个位置（环形回绕） */
    uint16_t idx = (uint16_t)((s_flow_pos + s_cfg.count - t) % s_cfg.count);
    uint8_t  pct = (uint8_t)(100u - (t * 100u) / WS_FLOW_TAIL);   /* 头部100% → 尾端≈0 */
    put_pixel(idx, scale_color(s_cfg.color, (uint8_t)((s_cfg.brightness * pct) / 100u)));
  }
  ws_flush();   /* 只下发实际需要长度，区域外保持熄灭 */
}
/* USER CODE END Private_Functions */

/**
  * @brief  初始化：启动输出后端，按默认配置渲染首帧
  */
void ws2812_init(void)
{
  ws2812_port_init();
  s_need_render = 1u;
  s_blink_on    = 0u;
  s_flow_pos    = 0u;
  s_t_frame     = HAL_GetTick();
  render_uniform(100u);        /* 默认常亮：上电即显示默认颜色 */
  s_need_render = 0u;
}

/**
  * @brief  Modbus 写寄存器后推送新配置（模组合法性由 Modbus 层校验，此处兜底钳位）
  */
void ws2812_set_cfg(const ws_cfg_t *cfg)
{
  if (cfg == 0)
  {
    return;
  }

  s_cfg.count      = (cfg->count > WS2812_LED_MAX) ? WS2812_LED_MAX : cfg->count;
  s_cfg.mode       = (cfg->mode  > WS_MODE_MAX)    ? WS_MODE_OFF  : cfg->mode;
  s_cfg.brightness = (cfg->brightness > 100u)      ? 100u         : cfg->brightness;
  s_cfg.color      = cfg->color;
  if (s_cfg.mode == WS_MODE_FLOW)
  {
    s_cfg.speed = cfg->speed;
    if (s_cfg.speed < WS_SPEED_FLOW_MIN)     { s_cfg.speed = WS_SPEED_FLOW_MIN; }
    if (s_cfg.speed > WS_SPEED_FLOW_MAX)     { s_cfg.speed = WS_SPEED_FLOW_MAX; }
    if (s_flow_pos >= s_cfg.count)           { s_flow_pos = 0u; }
  }
  else if (s_cfg.mode == WS_MODE_BREATH)
  {
    s_cfg.speed = cfg->speed;
    if (s_cfg.speed < WS_SPEED_BREATH_MIN)   { s_cfg.speed = WS_SPEED_BREATH_MIN; }
    if (s_cfg.speed > WS_SPEED_BREATH_MAX)   { s_cfg.speed = WS_SPEED_BREATH_MAX; }
  }
  else
  {
    s_cfg.speed = cfg->speed;   /* 其它模式不使用，保留寄存器值即可 */
  }

  s_need_render = 1u;   /* 下次 ws2812_task() 立即按新配置渲染 */
}

/**
  * @brief  主循环周期调用：按模式推进灯效（时间片驱动，非阻塞）
  * @note   常灭/常亮仅在配置变化时渲染一次；闪烁在相位翻转时渲染；
  *         流水按 speed 步进；呼吸按 WS_FRAME_MS 节拍重算亮度。
  */
void ws2812_task(void)
{
  uint32_t now = HAL_GetTick();

  switch (s_cfg.mode)
  {
    case WS_MODE_OFF:      /* 常灭 */
    case WS_MODE_SOLID:    /* 常亮 */
      if (s_need_render)
      {
        s_need_render = 0u;
        if (s_cfg.mode == WS_MODE_OFF)
        {
          render_uniform(0u);          /* 全黑 */
        }
        else
        {
          render_uniform(100u);        /* 配置亮度全亮 */
        }
      }
      break;

    case WS_MODE_BLINK:    /* 闪烁 1Hz：500ms 亮 / 500ms 灭 */
    {
      uint8_t phase = (uint8_t)((now / WS_BLINK_HALF_MS) & 1u);
      if (s_need_render || (phase != s_blink_on))
      {
        s_need_render = 0u;
        s_blink_on    = phase;
        render_uniform(phase ? 100u : 0u);
      }
    }
    break;

    case WS_MODE_FLOW:     /* 流水灯：每 speed(ms) 步进一颗 */
      if (s_need_render || (uint32_t)(now - s_t_frame) >= s_cfg.speed)
      {
        s_need_render = 0u;
        s_t_frame     = now;
        render_flow();
        if (s_cfg.count != 0u)   /* 灯数 0 时不做取模(避免除零/状态错乱) */
        {
          s_flow_pos = (uint16_t)((s_flow_pos + 1u) % s_cfg.count);
        }
      }
      break;

    case WS_MODE_BREATH:   /* 呼吸灯：三角波相位 + 平方 Gamma，全周期 speed(ms) */
      if (s_need_render || (uint32_t)(now - s_t_frame) >= WS_FRAME_MS)
      {
        s_need_render = 0u;
        s_t_frame     = now;

        /* 相位 0~199 → 三角波 tri 0~100 → Gamma(平方) 提亮中部、缓变两端 */
        uint32_t speed = (s_cfg.speed != 0u) ? s_cfg.speed : WS_SPEED_BREATH_MIN;
        uint32_t phase = ((now % speed) * 200u) / speed;
        uint32_t tri   = (phase < 100u) ? phase : (200u - phase);   /* 0~100 */
        uint8_t  pct   = (uint8_t)((tri * tri) / 100u);             /* 0~100 Gamma2.0 */

        render_uniform(pct);
      }
      break;

    default:               /* 未知模式兜底：熄灭 */
      if (s_need_render)
      {
        s_need_render = 0u;
        render_uniform(0u);
      }
      break;
  }
}


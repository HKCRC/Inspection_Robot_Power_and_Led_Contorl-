/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    di.c
  * @brief   数字量输入(DI)实现：IN1/IN2/IN3/TOUCH/START 五路输入
  * @note    全部低电平有效：输入为低(开关闭合/按下)时视为有效/触发。
  *          引脚由 CubeMX 配置(输入+上拉)，此处仅读取不做重复配置。
  *          状态经软件防抖后供 Modbus 寄存器(FC03/FC04)上报。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "di.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* 五路输入引脚映射：与 main.h 的 CubeMX 引脚宏保持一致
 * IN1_BLUE=PC8  IN2_WHITE=PC6  IN3_GREEN=PB15  TOUCH=PA4  START=PA8 */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
} di_map_t;

 const di_map_t s_di[DI_IN_MAX] =
{
  { IN1_C_GPIO_Port,     IN1_C_Pin     },   /* 0: IN1   */
  { IN2_B_GPIO_Port, 		 IN2_B_Pin     },   /* 1: IN2   */
  { IN3_A_GPIO_Port, 		 IN3_A_Pin     },   /* 2: IN3   */
  { IN0_TOUCH_GPIO_Port, IN0_TOUCH_Pin },   /* 3: TOUCH  机器人盖章   */
  { START_GPIO_Port,     START_Pin     },   /* 4: START */
};

/* 连续 N 次采样一致才确认(主循环 1ms 周期 ≈5ms 防抖) */
#define DI_DEBOUNCE_N    5u
/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
 uint8_t s_stable[DI_IN_MAX];   /* 防抖后的稳定状态 */
 uint8_t s_cand[DI_IN_MAX];     /* 候选状态 */
 uint8_t s_cnt[DI_IN_MAX];      /* 连续一致计数 */
/* USER CODE END Private variables */

/* USER CODE BEGIN Private_Function_Prototypes */
static uint8_t di_active(uint8_t lvl);
/* USER CODE END Private_Function_Prototypes */

/* USER CODE BEGIN Private_Functions */
/* 低电平有效：输入为低(闭合/按下)视为有效 */
static uint8_t di_active(uint8_t lvl)
{
  return (lvl == GPIO_PIN_RESET) ? 1 : 0;
}
/* USER CODE END Private_Functions */

/**
  * @brief  初始化并读取一次初值
  * @note   输入引脚由 CubeMX 配置，此处只读取不重复初始化
  */
void di_init(void)
{
  for (uint8_t i = 0u; i < DI_IN_MAX; i++)
  {
    s_stable[i] = di_active(HAL_GPIO_ReadPin(s_di[i].port, s_di[i].pin));
    s_cand[i]   = s_stable[i];
    s_cnt[i]    = 1u;
  }
}

/**
  * @brief  主循环周期调用：采样 + 软件防抖
  */
void di_scan(void)
{
  for (uint8_t i = 0u; i < DI_IN_MAX; i++)
  {
    uint8_t v = di_active(HAL_GPIO_ReadPin(s_di[i].port, s_di[i].pin));
    if (v == s_cand[i])
    {
      if (++s_cnt[i] >= DI_DEBOUNCE_N) { s_stable[i] = v; s_cnt[i] = 1u; }
    }
    else
    {
			
      s_cand[i] = v;
      s_cnt[i]  = 1u;
    }
  }
}

/**
  * @brief  取单路输入状态
  * @param  idx : 0=IN1 1=IN2 2=IN3 3=TOUCH 4=START
  * @retval 1=触发(低电平)  0=未触发；idx 非法返回 0
  */
uint8_t di_get_input(uint8_t idx)
{
  return (idx < DI_IN_MAX) ? s_stable[idx] : 0u;
}

/**
  * @brief  五路输入打包位图
  * @retval bit0=IN1 bit1=IN2 bit2=IN3 bit3=TOUCH bit4=START，1=触发
  */
uint16_t di_get_packed(void)
{
  uint16_t m = 0u;
  for (uint8_t i = 0u; i < DI_IN_MAX; i++)
  {
    if (s_stable[i] != 0u) { m |= (uint16_t)(1u << i); }
  }
  return m;
}

/* ---- 兼容旧控制逻辑(ctl.c/led.c/FC02)的映射 ---- */
uint8_t di_get_limit_left(void)  { return 0u; }                /* 限位已取消 */
uint8_t di_get_limit_right(void) { return 0u; }
uint8_t di_get_limit(void)       { return 0u; }
uint8_t di_get_touch(void)       { return di_get_input(3u); }  /* TOUCH(机器人盖章) 状态；原名 di_get_estop() */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

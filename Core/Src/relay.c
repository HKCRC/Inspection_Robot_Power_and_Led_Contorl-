/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    relay.c
  * @brief   数字输出(DO)统一驱动实现
  * @note    仅做 通道号->引脚 的映射与电平读写，输出策略(什么时候该输出什么)
  *          全部在 ctl.c(按键联动/电源锁存) 与 modbus_rtu.c(上位机线圈) 中，
  *          保证本层与业务零耦合。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "relay.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* 输出通道引脚映射表：第 N 路对应 s_do_map[N-1]，顺序与 relay.h 通道宏一致 */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
} relay_map_t;

static const relay_map_t s_do_map[RELAY_CH_MAX] =
{
  { OUT_jidianqi_GPIO_Port, OUT_jidianqi_Pin },    /* 1: 继电器        PC9  */
  { OUT_MOS_GPIO_Port,         OUT_MOS_Pin   },    /* 2: MOS管(主电源) PC12 */
  { OUT1_C_GPIO_Port,   			  OUT1_C_Pin   },    /* 3: 联动输出1(蓝) PE10 */
  { OUT2_B_GPIO_Port,  				   OUT2_B_Pin  },    /* 4: 联动输出2(白) PB1  */
  { OUT3_A_GPIO_Port, 				   OUT3_A_Pin  },    /* 5: 联动输出3(绿) PA6  */
  { OUT0_START_GPIO_Port,    OUT0_START_Pin  },    /* 6: 联动输出4     PE11 */
};

/* USER CODE BEGIN Private_Function_Prototypes */

/* USER CODE END Private_Function_Prototypes */

/* USER CODE BEGIN Private_Functions */

/* USER CODE END Private_Functions */

/**
  * @brief  初始化，上电默认全部输出低（继电器断开 / MOS 关断 / 联动输出无效）
  */
void relay_init(void)
{
  for (uint8_t i = 0; i < RELAY_CH_MAX; i++)
  {
    HAL_GPIO_WritePin(s_do_map[i].port, s_do_map[i].pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief  单路输出控制（上层 Modbus/联动逻辑的底层实现）
  * @param  ch : 通道号，从 1 开始（见 relay.h 通道宏）
  * @param  on : RELAY_ON=输出高  RELAY_OFF=输出低
  * @retval 1 成功  0 通道非法
  */
uint8_t relay_set(uint8_t ch, uint8_t on)
{
  if (ch == 0 || ch > RELAY_CH_MAX)
  {
    return 0;
  }
  HAL_GPIO_WritePin(s_do_map[ch - 1].port, s_do_map[ch - 1].pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return 1;
}

/**
  * @brief  查询所有输出通道状态
  * @retval 位图，bitN 对应第 N+1 路，1=输出高
  */
uint16_t relay_get_state(void)
{
  uint16_t st = 0;
  for (uint8_t i = 0; i < RELAY_CH_MAX; i++)
  {
    if (HAL_GPIO_ReadPin(s_do_map[i].port, s_do_map[i].pin) == GPIO_PIN_SET)
    {
      st |= (uint16_t)(1u << i);
    }
  }
  return st;
}

/* USER CODE BEGIN Relay_UserFunctions */

/* 扩展继电器逻辑（如互锁/延时/联动）在此添加 */

/* USER CODE END Relay_UserFunctions */

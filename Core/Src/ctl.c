/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ctl.c
  * @brief   输出控制逻辑层实现：按键联动输出 + 电源(MOS)锁存状态机
  * @note    职责边界（对应需求：需求资源分配.md）：
  *          ① IN1_BLUE  按下 -> OUT1_BULE  高电平（电平跟随，松开回落）
  *          ② IN2_WHITE 按下 -> OUT2_WHITE 高电平
  *          ③ IN3_GREEN 按下 -> OUT3_GREEN 高电平
  *          ④ START     检测到低电平(按下) -> OUT_START 高电平并锁存保持
  *             （按键无自锁，松开不回落），随上位机断电指令一并复位
  *          ⑤ START     检测到低电平(按下) -> OUT_MOS 高电平，MOS 通电
  *             （按下即上电，含开机时已按住的场景；本地只能开机）
  *          ⑥ 通电后断电只响应上位机"写线圈0=0x0000"指令
  *          ⑦ 继电器(线圈1)：吸合指令立即执行；关闭指令延时15s再输出高
  *             电平断开触点（继电器驱动为低电平吸合，极性宏可改）
  *          输入统一取自 di.c 防抖稳定状态，输出统一经 relay.c 通道驱动，
  *          本模块不直接操作 GPIO，与硬件引脚解耦。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "ctl.h"
#include "relay.h"
#include "di.h"

/* USER CODE BEGIN Private defines */
/* di.c 输入通道索引（与 di.c 的 s_di[] 顺序一致，仅此处引用，便于集中调整） */
#define CTL_DI_IN1      0u   /* IN1_BLUE  */
#define CTL_DI_IN2      1u   /* IN2_WHITE */
#define CTL_DI_IN3      2u   /* IN3_GREEN */
#define CTL_DI_START    4u   /* START（3=TOUCH，本模块不使用） */

/* 继电器驱动极性：硬件低电平吸合（输出低=触点闭合，输出高=触点断开）。
 * 极性只在此处适配，relay.c 保持纯电平语义；若实测相反改这两个宏即可 */
#define CTL_RELAY_PULLIN_LVL     RELAY_OFF   /* 吸合 = 输出低 */
#define CTL_RELAY_RELEASE_LVL    RELAY_ON    /* 关闭 = 输出高 */

#define CTL_RELAY_CLOSE_DELAY_MS  5000u     /* 关闭指令->执行断开的延时 5s */
/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
static uint8_t s_power;            /* 电源锁存状态：1=MOS通电 0=断电 */
static uint8_t s_relay;            /* 继电器逻辑状态：1=吸合(输出低) 0=关闭(输出高) */
static uint8_t s_relay_close_pend; /* 1=继电器关闭延时进行中(15s 计时) */
static uint32_t s_relay_close_t0;  /* 继电器关闭指令到达时刻(ms tick) */
/* USER CODE END Private variables */

/* USER CODE BEGIN Private_Function_Prototypes */

/* USER CODE END Private_Function_Prototypes */

/**
  * @brief  初始化
  * @note   上电默认断电(MOS 关断)：若开机时 START 已被按住，首个 ctl_task
  *         周期即检测到低电平并通电（按下即上电）。
  */
void ctl_init(void)
{
  s_power            = 0u;   /* MOS 关断，等待 START 按下或上位机指令 */
  s_relay            = 1u;   /* 上电默认吸合(与 relay_init 输出低=低吸合一致) */
  s_relay_close_pend = 0u;
}

/**
  * @brief  上位机写线圈0：电源通/断指令入口（Modbus FC05 调用）
  * @param  on  1=MOS通电  0=断电
  * @note   断电是本地唯一无法完成的操作，只能由上位机经此接口下发。
  */
void ctl_set_power(uint8_t on)
{
  s_power = on ? 1u : 0u;
}

/**
  * @brief  查询当前电源锁存状态（Modbus FC01 读线圈0 的数据源）
  * @retval 1=通电 0=断电
  */
uint8_t ctl_power_get(void)
{
  return s_power;
}

/**
  * @brief  上位机写线圈1：继电器控制指令入口（Modbus FC05 调用）
  * @param  on  1=吸合(立即执行，取消挂起的延时关闭)  0=关闭(延时15s断开触点)
  */
void ctl_set_relay(uint8_t on)
{
  if (on)
  {
    s_relay            = 1u;
    s_relay_close_pend = 0u;            /* 吸合指令取消未完成的延时关闭 */
  }
  else
  {
    s_relay            = 0u;            /* 逻辑状态立即更新(读线圈即返回关闭) */
    s_relay_close_pend = 1u;            /* 启动 15s 延时，ctl_task 中到点执行 */
    s_relay_close_t0   = HAL_GetTick();
  }
}

/**
  * @brief  查询继电器逻辑状态（Modbus FC01 读线圈1 的数据源）
  * @retval 1=吸合 0=关闭(含延时进行中)
  */
uint8_t ctl_relay_get(void)
{
  return s_relay;
}

/**
  * @brief  主循环周期调用：刷新按键联动输出与电源锁存
  * @note   电平跟随输出每周期无条件刷新，抗干扰自恢复；
  *         电源锁存为电平触发+软件锁存：检测到 START 按下即置位，
  *         松开后保持（按键无自锁），只有上位机断电指令能清零。
  */
void ctl_task(void)
{
  uint8_t start_now = di_get_input(CTL_DI_START);

  /* ① 按键联动输出：IN1/IN2/IN3 电平跟随对应按键（按下=高，松开=低） */
  relay_set(RELAY_CH_OUT1_BULE,  di_get_input(CTL_DI_IN1));
  relay_set(RELAY_CH_OUT2_WHITE, di_get_input(CTL_DI_IN2));
  relay_set(RELAY_CH_OUT3_GREEN, di_get_input(CTL_DI_IN3));

  /* ② 电源锁存：检测到 START 低电平(按下)即置位上电——按下时才上电，
   *    松开后软件锁存保持（按键无自锁）；清零路径只有一条 ——
   *    上位机写线圈0(ctl_set_power)，本地按键不能断电 */
  if (start_now)
  {
    s_power = 1u;
  }

  /* ③ 锁存输出：OUT_START 与 MOS 电源同态——检测到按下即置高并保持，
   *    松开不回落；上位机写线圈0断电时两路一并拉低复位 */
  relay_set(RELAY_CH_MOS,       s_power);
  relay_set(RELAY_CH_OUT_START, s_power);

  /* ④ 继电器延时控制：吸合立即执行；关闭指令延时15s再输出高电平断开触点 */
  if (s_relay)
  {
    relay_set(RELAY_CH_JIDIANQI, CTL_RELAY_PULLIN_LVL);   /* 吸合：输出低 */
  }
  else if (s_relay_close_pend &&
           (HAL_GetTick() - s_relay_close_t0) >= CTL_RELAY_CLOSE_DELAY_MS)
  {
    s_relay_close_pend = 0u;
    relay_set(RELAY_CH_JIDIANQI, CTL_RELAY_RELEASE_LVL);  /* 关闭：输出高 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

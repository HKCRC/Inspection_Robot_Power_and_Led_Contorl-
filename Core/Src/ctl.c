/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ctl.c
  * @brief   输出控制逻辑层实现：按键联动输出 + 电源(MOS)锁存/延时断电状态机
  * @note    职责边界（对应需求：需求资源分配.md）：
  *          ① IN1_BLUE  按下 -> OUT1_BULE  高电平（电平跟随，松开回落）
  *          ② IN2_WHITE 按下 -> OUT2_WHITE 高电平
  *          ③ IN3_GREEN 按下 -> OUT3_GREEN 高电平
  *          ④ START     检测到低电平(按下) -> OUT_START 高电平并锁存保持
  *             （按键无自锁，松开不回落），随上位机断电指令一并复位
  *          ⑤ START     检测到低电平(按下) -> OUT_MOS 高电平，MOS 通电
  *             （按下即上电，含开机时已按住的场景；本地只能开机）
  *          ⑥ 断电只响应上位机"写线圈0=0x0000"，而且是**延时断 MOS**：
  *             收到关电指令后 MOS 继续供电 CTL_POWER_OFF_DELAY_MS(15s)，
  *             给上位机留出关机时间；期间再写线圈0=0xFF00 可取消延时。
  *             注意：读线圈0 上报的是**命令状态**（写完 0 立即读回 0），
  *             不是 MOS 的实际输出（延时期间仍在通电）。
  *          ⑦ 继电器(线圈1)：吸合/断开均**立即**执行，无延时
  *             （继电器驱动为低电平吸合，极性宏可改）
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
#define CTL_RELAY_RELEASE_LVL    RELAY_ON    /* 断开 = 输出高 */

/* 关电指令 -> 断开 MOS 的延时：这段时间 MOS 仍供电，供上位机完成关机 */
#define CTL_POWER_OFF_DELAY_MS  15000u
/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
static uint8_t  s_power_cmd;       /* 电源命令状态：1=通电命令 0=关电命令(FC01 读线圈0 数据源) */
static uint8_t  s_power_out;       /* MOS 实际输出：1=通电 0=断电(关电延时到期后才置 0) */
static uint8_t  s_power_off_pend;  /* 1=延时断电进行中(此时 s_power_out 仍为 1) */
static uint32_t s_power_off_t0;    /* 收到关电指令的时刻(ms tick) */
static uint8_t  s_relay;           /* 继电器状态：1=吸合(输出低) 0=断开(输出高)，无延时 */
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
  s_power_cmd      = 0u;   /* 命令状态：默认关电 */
  s_power_out      = 0u;   /* MOS 关断，等待 START 按下或上位机指令 */
  s_power_off_pend = 0u;
  s_relay          = 1u;   /* 上电默认吸合(与 relay_init 输出低=吸合一致) */
}

/**
  * @brief  上位机写线圈0：电源通/断指令入口（Modbus FC05 调用）
  * @param  on  1=通电(立即)  0=关电(延时 CTL_POWER_OFF_DELAY_MS 后断开 MOS)
  * @note   断电是本地唯一无法完成的操作，只能由上位机经此接口下发；
  *         关电不是立即断电：MOS 继续保持供电 15s，让上位机执行 shutdown，
  *         这期间再写线圈0=1 可取消延时。
  */
void ctl_set_power(uint8_t on)
{
  s_power_cmd = on ? 1u : 0u;           /* 命令状态立即更新：写 0 后立即读回 0 */

  if (on)
  {
    s_power_out      = 1u;
    s_power_off_pend = 0u;              /* 通电指令取消未完成的延时断电 */
  }
  else if (s_power_out != 0u)
  {
    s_power_off_pend = 1u;              /* MOS 仍通电，启动延时，ctl_task 到点断开 */
    s_power_off_t0   = HAL_GetTick();
  }
}

/**
  * @brief  查询电源命令状态（Modbus FC01 读线圈0 的数据源）
  * @retval 1=通电命令  0=关电命令
  * @note   表示"命令已被接受"：上位机写线圈0=0x0000 后立即读回 0；
  *         此时 MOS 仍在延时供电 CTL_POWER_OFF_DELAY_MS，MOS 实际输出
  *         由 ctl.c 内部的 s_power_out 跟踪（不对外上报）。
  */
uint8_t ctl_power_get(void)
{
  return s_power_cmd;
}

/**
  * @brief  上位机写线圈1：继电器控制指令入口（Modbus FC05 调用）
  * @param  on  1=吸合  0=断开（均立即执行，无延时）
  */
void ctl_set_relay(uint8_t on)
{
  s_relay = on ? 1u : 0u;
}

/**
  * @brief  查询继电器状态（Modbus FC01 读线圈1 的数据源）
  * @retval 1=吸合 0=断开
  */
uint8_t ctl_relay_get(void)
{
  return s_relay;
}

/**
  * @brief  主循环周期调用：刷新按键联动输出、电源锁存与延时断电
  * @note   电平跟随输出每周期无条件刷新，抗干扰自恢复；
  *         电源为电平触发+软件锁存：检测到 START 按下即置位，松开后保持
  *         （按键无自锁），只有上位机关电指令能清零。
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
   *    上位机写线圈0(ctl_set_power)。延时断电进行中忽略 START
   *    （需求 §4.2：关机倒计时期间 START 按键不处理） */
  if ((start_now != 0u) && (s_power_off_pend == 0u))
  {
    s_power_cmd = 1u;
    s_power_out = 1u;
  }

  /* ③ 延时断电到点：此刻才真正断开 MOS（此前一直保持供电） */
  if ((s_power_off_pend != 0u) &&
      ((HAL_GetTick() - s_power_off_t0) >= CTL_POWER_OFF_DELAY_MS))
  {
    s_power_off_pend = 0u;
    s_power_out      = 0u;
  }

  /* ④ 锁存输出：OUT_START 与 MOS 电源同态——检测到按下即置高并保持，
   *    松开不回落；上位机关电延时到期后两路一并拉低复位 */
  relay_set(RELAY_CH_MOS,       s_power_out);
  relay_set(RELAY_CH_OUT_START, s_power_out);

  /* ⑤ 继电器：吸合/断开均立即生效，无延时 */
  relay_set(RELAY_CH_JIDIANQI,
            (s_relay != 0u) ? CTL_RELAY_PULLIN_LVL : CTL_RELAY_RELEASE_LVL);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

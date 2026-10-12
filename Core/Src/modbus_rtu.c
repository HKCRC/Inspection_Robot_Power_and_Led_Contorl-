/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    modbus_rtu.c
  * @brief   Modbus RTU 从机实现
  *          - 帧接收：串口 DMA + 空闲中断(IDLE) 判定一帧结束
  *            串口由 MB_USE_UART4 宏选择（0=USART1  1=UART4）
  *          - 协议：CRC16 校验、从机地址匹配、标准功能码
  *          - 控制：0x05 写单线圈 -> 线圈0=电源(MOS,经ctl锁存)
  *            线圈1=继电器(吸合立即/关闭延时15s，策略在ctl.c)
  *            0x01 读线圈 -> 回读电源锁存状态与继电器逻辑状态
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "modbus_rtu.h"
#include "usart.h"
#include "relay.h"
#include "stm32h7xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "di.h"
#include "ctl.h"
#include "ws2812.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* ---- Modbus 通信串口选择宏 ------------------------------------------------
 * MB_USE_UART4 = 0 : 使用 USART1（PA9=TX  / PA10=RX）
 * MB_USE_UART4 = 1 : 使用 UART4 （PA12=TX / PA11=RX）
 * 只改这个宏即可整体切换串口，函数体无需改动
 * ------------------------------------------------------------------------ */
#define MB_USE_UART4   0u

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;

#if MB_USE_UART4
  #define MB_UART       (&huart4)   /* Modbus 收发使用的串口句柄 */
  #define MB_UART_INST  UART4       /* 回调中匹配用的外设实例 */
#else
  #define MB_UART       (&huart1)
  #define MB_UART_INST  USART1
#endif

#define MB_SLAVE_ADDR       0x08u   /* 从机地址 */
#define MB_RX_BUF_SIZE      64      /* 接收缓冲（每帧独立，Normal 模式） */
#define MB_TX_BUF_SIZE      64

#define MB_FC_READ_COILS           0x01u
#define MB_FC_READ_DISCRETE        0x02u   /* 读离散输入：bit0=IN1..bit4=START */
#define MB_FC_READ_HOLDING         0x03u   /* 读保持寄存器：五路输入 */
#define MB_FC_READ_INPUT_REG       0x04u   /* 读输入寄存器：五路输入(只读) */
#define MB_FC_WRITE_SINGLE_COIL    0x05u
#define MB_FC_WRITE_SINGLE_REG     0x06u   /* 写单寄存器：WS2812 配置 */
#define MB_FC_WRITE_MULTI_REG      0x10u   /* 写多寄存器：批量写 WS2812 配置 */

#define MB_EXC_ILLEGAL_FUNCTION    0x01u
#define MB_EXC_ILLEGAL_ADDRESS     0x02u
#define MB_EXC_ILLEGAL_VALUE       0x03u

/* ---- 输入寄存器映射(FC03/FC04 共用) --------------------------------------
 * 0x0000=IN1   0x0001=IN2   0x0002=IN3   0x0003=TOUCH  0x0004=START
 * 0x0005=打包位图 bit0=IN1 bit1=IN2 bit2=IN3 bit3=TOUCH bit4=START
 * 寄存器值：1=触发(输入低电平)  0=未触发(数据源为 di.c 防抖后的状态)
 * ------------------------------------------------------------------------ */
/* ---- WS2812 保持寄存器(FC03/04 读，FC06/16 写) ----------------------------
 * 0x0006=灯数(0~160)     0x0007=亮度%(0~100)  0x0008/9/A=R/G/B(0~255)
 * 0x000B=灯效模式        0=常灭 1=常亮 2=闪烁1Hz 3=流水灯 4=呼吸灯
 * 0x000C=速度参数        流水=步进间隔ms(10~1000) 呼吸=全周期ms(500~10000)
 * 上电默认 160 灯/红色/亮度100%/常亮（与 ws2812.c 引擎默认值一致）
 * 写入成功后经 ws2812_set_cfg() 推送给灯效引擎即时生效
 * ------------------------------------------------------------------------ */
#define MB_REG_COUNT      13u   /* 0x0000~0x0005 输入  0x0006~0x000C WS2812 配置 */
#define MB_WS_REG_BASE     6u
#define MB_WS_REG_COUNT    7u

/* ---- 线圈映射(FC01 读 / FC05 写) ------------------------------------------
 * 线圈0 = 电源(MOS管)   0xFF00=通电(立即) 0x0000=关电(延时15s再断 MOS，见 ctl.c)
 *         电源锁存状态由 ctl.c 管理：START 按键也可本地通电
 * 线圈1 = 继电器        0xFF00=吸合 0x0000=断开（均立即执行，完全由上位机控制）
 * 联动输出(OUT1~OUT3/OUT_START)跟随本地按键，不开放线圈控制
 * ------------------------------------------------------------------------ */
#define MB_COIL_COUNT  2u

 uint8_t  s_rx_buf[MB_RX_BUF_SIZE];
 uint8_t  s_tx_buf[MB_TX_BUF_SIZE];
 /* WS2812 寄存器镜像：[0]=灯数 [1]=亮度 [2]=R [3]=G [4]=B [5]=模式 [6]=速度
  * 默认 160 灯/100%/红/常亮/流水默认速度 */
 volatile uint16_t s_ws_regs[MB_WS_REG_COUNT] = {160u, 100u, 0xFFu, 0x00u, 0x00u, WS_MODE_SOLID, WS_SPEED_FLOW_DEF};
/* ---- 接收缓冲与诊断变量 --------------------------------------------------
 * s_rx_dma_buf：DMA 直接写入；s_rx_buf：中断里拷出的整帧，主循环解析用。
 * 两者分离，主循环解析时 DMA 已经在收下一帧，互不干扰。
 * ------------------------------------------------------------------------ */
static uint8_t  s_rx_dma_buf[MB_RX_BUF_SIZE];
static volatile uint8_t s_rx_armed;      /* 1=接收 DMA 已武装 */
volatile uint32_t s_mb_rx_evt_cnt;       /* 诊断：完整帧(IDLE/TC)次数 */
volatile uint32_t s_mb_rx_ht_cnt;        /* 诊断：半传输事件次数(已被忽略) */
volatile uint32_t s_mb_rx_err_cnt;       /* 诊断：接收错误次数(ORE/FE/NE/DMA) */
volatile uint32_t s_mb_rx_rearm_cnt;     /* 诊断：接收(重新)武装成功次数 */
volatile uint8_t  s_mb_rx_err_code;      /* 诊断：最近一次 huart->ErrorCode */
static volatile uint16_t s_rx_len;      /* 最近一帧字节数 */
static volatile uint8_t  s_rx_ready;    /* 帧接收完成标志 */
static volatile uint8_t  s_tx_err;      /* 诊断：应答发送失败标志(0=成功) */

/* USER CODE END PV */

/* USER CODE BEGIN Private_Function_Prototypes */
static uint16_t mb_crc16(const uint8_t *data, uint16_t len);
static uint16_t mb_build_read_coils(uint16_t start, uint16_t count);
static uint16_t mb_build_read_regs(uint16_t start, uint16_t count);
static uint16_t mb_build_write_single_coil(uint16_t start, uint16_t val);
static uint16_t mb_build_exception(uint8_t func, uint8_t excode);
static uint8_t  mb_ws_reg_valid(uint8_t ofs, uint16_t val);
static void     mb_ws_push_cfg(void);
/* USER CODE END Private_Function_Prototypes */

/* USER CODE BEGIN Private_Functions */
/**
  * @brief  Modbus RTU CRC16
  */
static uint16_t mb_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x0001u)
      {
        crc = (crc >> 1) ^ 0xA001u;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/**
  * @brief  组装"读线圈"响应帧，返回帧长度；失败返回 0
  * @note   线圈0=电源(MOS)锁存状态(数据源 ctl.c)，线圈1=继电器实际输出电平
  * @note   s_tx_buf[0..1] 已填好 从机地址+功能码
  */
static uint16_t mb_build_read_coils(uint16_t start, uint16_t count)
{
  /* 校验地址/数量：线圈地址为 16 位(0x0000~0xFFFF)，用 uint32 求和防溢出；越界(含 0x0100 这类高字节非 0 的地址)统一按非法地址处理 */
  if (count == 0u || count > MB_COIL_COUNT || ((uint32_t)start + count) > MB_COIL_COUNT)
  {
    return 0;
  }

  uint8_t  byte_cnt = (uint8_t)((count + 7) / 8);   /* 每字节 8 个线圈 */
  uint16_t state    = (uint16_t)(ctl_power_get() |   /* bit0=电源(MOS)锁存状态 */
                    (ctl_relay_get() ? 0x2u : 0u));  /* bit1=继电器逻辑状态 */
  uint16_t idx      = 2;

  s_tx_buf[idx++] = byte_cnt;
  for (uint8_t b = 0; b < byte_cnt; b++)
  {
    uint8_t val = 0;
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      uint16_t coil = start + (uint16_t)(b * 8 + bit);
      if (coil >= start + count)
      {
        break;   /* 超出本次读取数量，剩余位补 0 */
      }
      if (((uint32_t)state & ((uint32_t)1u << coil)) != 0u)
      {
        val |= (uint8_t)(1u << bit);
      }
    }
    s_tx_buf[idx++] = val;
  }

  /* CRC(低位在前) */
  uint16_t crc = mb_crc16(s_tx_buf, idx);
  s_tx_buf[idx++] = (uint8_t)(crc & 0xFFu);
  s_tx_buf[idx++] = (uint8_t)(crc >> 8);
  return idx;
}

/**
  * @brief  组装"读寄存器"响应帧(FC03/FC04)，返回帧长度；失败返回 0
  * @note   寄存器 0x0000~0x0004 = 五路输入(0/1)，0x0005 = 打包位图
  * @note   s_tx_buf[0..1] 已填好 从机地址+功能码
  */
static uint16_t mb_build_read_regs(uint16_t start, uint16_t count)
{
  if (count == 0u || count > 125u || ((uint32_t)start + count) > MB_REG_COUNT)
  {
    return 0;
  }

  uint16_t idx = 2;
  s_tx_buf[idx++] = (uint8_t)(count * 2u);            /* 字节计数 = 寄存器数×2 */
  for (uint16_t r = 0u; r < count; r++)
  {
    uint16_t addr = (uint16_t)(start + r);
    uint16_t val;
    if (addr == 5u)   /* 0x0005：打包位图 */
    {
      val = di_get_packed();
    }
    else if (addr >= MB_WS_REG_BASE)   /* 0x0006~0x000A：WS2812 配置 */
    {
      val = s_ws_regs[addr - MB_WS_REG_BASE];
    }
    else              /* 0x0000~0x0004：单路输入，地址即 di_get_input 的 idx */
    {
      val = di_get_input((uint8_t)addr) ? 1u : 0u;
    }
    s_tx_buf[idx++] = (uint8_t)(val >> 8);              /* 寄存器高字节在前 */
    s_tx_buf[idx++] = (uint8_t)(val & 0xFFu);
  }

  /* CRC(低位在前) */
  uint16_t crc = mb_crc16(s_tx_buf, idx);
  s_tx_buf[idx++] = (uint8_t)(crc & 0xFFu);
  s_tx_buf[idx++] = (uint8_t)(crc >> 8);
  return idx;
}

/**
  * @brief  组装"写单线圈"响应帧（原样回显请求），返回帧长度；失败返回 0
  * @note   线圈0 -> ctl_set_power()：电源(MOS)通/断，0x0000 即"关电"指令
  *         线圈1 -> 继电器吸合/断开
  */
static uint16_t mb_build_write_single_coil(uint16_t start, uint16_t val)
{
  if (start >= MB_COIL_COUNT)
  {
    return 0;
  }
  /* 标准值：0xFF00=吸合/通电 0x0000=断开/断电 */
  if (val != 0xFF00u && val != 0x0000u)
  {
    return 0;
  }

  uint8_t on = (val == 0xFF00u) ? 1u : 0u;
  if (start == 0u)       /* 线圈0：电源(MOS) -> 交由控制逻辑层锁存 */
  {
    ctl_set_power(on);
  }
  else                   /* 线圈1：继电器 -> 吸合立即/关闭延时15s，策略在 ctl.c */
  {
    ctl_set_relay(on);
  }

  /* 响应 = 请求回显 */
  uint16_t idx = 2;
  s_tx_buf[idx++] = (uint8_t)(start >> 8);
  s_tx_buf[idx++] = (uint8_t)(start & 0xFFu);
  s_tx_buf[idx++] = (uint8_t)(val >> 8);
  s_tx_buf[idx++] = (uint8_t)(val & 0xFFu);
  uint16_t crc = mb_crc16(s_tx_buf, idx);
  s_tx_buf[idx++] = (uint8_t)(crc & 0xFFu);
  s_tx_buf[idx++] = (uint8_t)(crc >> 8);
  return idx;
}

/**
  * @brief  组装异常响应帧，返回帧长度
  */
static uint16_t mb_build_exception(uint8_t func, uint8_t excode)
{
  uint16_t idx = 2;
  s_tx_buf[1]  = func | 0x80u;   /* 异常功能码 */
  s_tx_buf[idx++] = excode;
  uint16_t crc = mb_crc16(s_tx_buf, idx);
  s_tx_buf[idx++] = (uint8_t)(crc & 0xFFu);
  s_tx_buf[idx++] = (uint8_t)(crc >> 8);
  return idx;
}

/**
  * @brief  校验 WS2812 配置寄存器写入值是否合法
  * @param  ofs  寄存器偏移 0=灯数 1=亮度 2/3/4=R/G/B 5=模式 6=速度
  * @retval 1=合法 0=超范围(触发 0x03 异常)
  */
static uint8_t mb_ws_reg_valid(uint8_t ofs, uint16_t val)
{
  if (ofs == 0u)                       /* 灯数上限 = 灯带容量(ws2812_port.h) */
  {
    return (val <= WS2812_LED_MAX) ? 1u : 0u;
  }
  if (ofs == 1u)                       /* 亮度上限 100% */
  {
    return (val <= 100u) ? 1u : 0u;
  }
  if (ofs == 5u)                       /* 灯效模式 0~4 */
  {
    return (val <= WS_MODE_MAX) ? 1u : 0u;
  }
  if (ofs == 6u)                       /* 速度参数上限 10000ms（下限由引擎钳位） */
  {
    return (val <= 10000u) ? 1u : 0u;
  }
  return (val <= 255u) ? 1u : 0u;      /* R/G/B 上限 255 */
}

/**
  * @brief  将 WS2812 寄存器镜像(0x0006~0x000C)推送给灯效引擎，写入后调用
  */
static void mb_ws_push_cfg(void)
{
  ws_cfg_t cfg;

  cfg.count      = s_ws_regs[0];
  cfg.brightness = (uint8_t)s_ws_regs[1];
  cfg.color      = ((uint32_t)s_ws_regs[2] << 16)
                 | ((uint32_t)s_ws_regs[3] << 8)
                 |  (uint32_t)s_ws_regs[4];
  cfg.mode       = (uint8_t)s_ws_regs[5];
  cfg.speed      = s_ws_regs[6];
  ws2812_set_cfg(&cfg);
}

/**
  * @brief  武装/重新武装 Modbus 接收(DMA + IDLE 变长帧)
  * @retval 1=成功  0=失败(未武装，由 modbus_task 兜底重试)
  * @note   可在中断上下文调用：回调里立刻重新武装，把"DMA 未武装窗口"压到微秒级，
  *         避免接收溢出(ORE)打死接收(NULL/UART 状态不对时返回 0，不阻塞)。
  */
static uint8_t mb_rx_start(void)
{
  HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(MB_UART, s_rx_dma_buf, MB_RX_BUF_SIZE);

  if (st == HAL_OK)
  {
    s_rx_armed = 1u;
    s_mb_rx_rearm_cnt++;
  }
  else if (st == HAL_BUSY)
  {
    /* 已有接收在进行(不应发生)：按实际状态判定，避免误判为"未武装"后反复重试 */
    s_rx_armed = (MB_UART->RxState == HAL_UART_STATE_BUSY_RX) ? 1u : 0u;
  }
  else
  {
    s_rx_armed = 0u;   /* HAL_ERROR：交由 modbus_task 健康检查重试 */
  }
  return s_rx_armed;
}
/* USER CODE END Private_Functions */

/**
  * @brief  Modbus 串口 DMA 空闲中断回调：一帧接收完成（实例由 MB_UART_INST 匹配）
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == MB_UART_INST)
  {
    /* 半传输(HT)事件不是"一帧结束"：此时接收仍在进行，忽略，避免把 32 字节误当整帧 */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    {
      s_mb_rx_ht_cnt++;
      return;
    }

    if (Size > MB_RX_BUF_SIZE)
    {
      Size = MB_RX_BUF_SIZE;
    }
    if (Size > 0u)
    {
      memcpy(s_rx_buf, s_rx_dma_buf, Size);   /* 拷出整帧：主循环解析不再与 DMA 抢缓冲 */
      s_rx_len   = Size;
      s_rx_ready = 1u;
      s_mb_rx_evt_cnt++;
    }

    /* 立刻重新武装接收：把"DMA 未武装窗口"压到微秒级，避免溢出(ORE)打死接收 */
    (void)mb_rx_start();
  }
}

/**
  * @brief  初始化并启动 Modbus 接收
  */
void modbus_init(void)
{
  s_rx_len   = 0;
  s_rx_ready = 0;
  (void)mb_rx_start();
}

/**
  * @brief  主循环任务：轮询上位机帧，处理并发送响应（非阻塞）
  * @note   接收由 DMA+IDLE 在后台完成（填 s_rx_buf 并置 s_rx_ready）。
  *         反复调用本函数：没有新帧时立即返回 0，不打结主循环；
  *         收到"上位机关电"指令(写线圈0=0x0000)时返回 1，供上层做指示等处理。
  * @retval 1  收到关电指令
  * @retval 0  无新帧 / 其它类型帧 / 帧不可用(长度不足/地址不符/CRC失败)
  */
uint8_t modbus_task(void)
{
  uint16_t resp_len;
  uint8_t  close_flag = 0;

  /* 轮询：无新帧则立即返回，不阻塞 */
  if (!s_rx_ready)
  {
    return 0;
  }
  s_rx_ready = 0;

  /* 接收健康检查：正常情况下中断回调已立刻重新武装；这里兜底——
     若接收 DMA 未武装(被错误中止/武装失败)，重新武装，避免通讯永久失效 */
  if ((s_rx_armed == 0u) && (MB_UART->RxState == HAL_UART_STATE_READY))
  {
    (void)mb_rx_start();
  }

  /* 最小帧：addr + func + crc(2) = 4 字节 */
  if (s_rx_len < 4)
  {
    return 0;
  }

  /* 从机地址匹配（0xFF 广播不回响应） */
  if (s_rx_buf[0] != MB_SLAVE_ADDR && s_rx_buf[0] != 0xFFu)
  {
    return 0;
  }

  /* CRC 校验（低字节在前） */
  {
    uint16_t crc_recv = (uint16_t)(s_rx_buf[s_rx_len - 2] | (s_rx_buf[s_rx_len - 1] << 8));
    if (mb_crc16(s_rx_buf, s_rx_len - 2) != crc_recv)
    {
      return 0;
    }
  }

  /* 收到有效帧（通信诊断/联动策略见 ctl.c，此处不再做看门狗） */

  /* 广播请求已检查通过，不回复（不是关闭指令，返回 0） */
  if (s_rx_buf[0] == 0xFFu)
  {
    return 0;
  }

  s_tx_buf[0] = MB_SLAVE_ADDR;
  s_tx_buf[1] = s_rx_buf[1];

  switch (s_rx_buf[1])
  {
    case MB_FC_READ_DISCRETE:    /* 02 读离散输入：bit0=IN1..bit4=START */
      if (s_rx_len < 8) return 0;
      s_tx_buf[2] = 0x01;                 /* 字节计数(5路输入占1字节) */
      s_tx_buf[3] = (uint8_t)(di_get_packed() & 0x1Fu);
      resp_len = 4;
      {
        uint16_t crc = mb_crc16(s_tx_buf, resp_len);
        s_tx_buf[resp_len++] = (uint8_t)(crc & 0xFFu);
        s_tx_buf[resp_len++] = (uint8_t)(crc >> 8);
      }
      break;

    case MB_FC_READ_HOLDING:     /* 03 读保持寄存器：五路输入 */
    case MB_FC_READ_INPUT_REG:   /* 04 读输入寄存器：五路输入 */
      if (s_rx_len < 8) return 0;
      resp_len = mb_build_read_regs(
          (uint16_t)((s_rx_buf[2] << 8) | s_rx_buf[3]),
          (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]));
      if (resp_len == 0)
      {
        resp_len = mb_build_exception(s_rx_buf[1], MB_EXC_ILLEGAL_ADDRESS);
      }
      break;

    case MB_FC_READ_COILS:       /* 01 读线圈 */
      if (s_rx_len < 8) return 0;
      resp_len = mb_build_read_coils(
          (uint16_t)((s_rx_buf[2] << 8) | s_rx_buf[3]),
          (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]));
      if (resp_len == 0)
      {
        resp_len = mb_build_exception(MB_FC_READ_COILS, MB_EXC_ILLEGAL_ADDRESS);
      }
      break;

    case MB_FC_WRITE_SINGLE_COIL: /* 05 写单线圈 */
      if (s_rx_len < 8) return 0;
      {
        uint16_t coil = (uint16_t)((s_rx_buf[2] << 8) | s_rx_buf[3]);
        uint16_t val  = (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]);
        resp_len = mb_build_write_single_coil(coil, val);
        if (resp_len == 0)
        {
          resp_len = mb_build_exception(MB_FC_WRITE_SINGLE_COIL, MB_EXC_ILLEGAL_ADDRESS);
        }
        else if (coil == 0u && val == 0u)   /* 线圈0=电源 且 断电：上位机"关电"指令 */
        {
          close_flag = 1;
        }
      }
      break;

    case MB_FC_WRITE_SINGLE_REG: /* 06 写单寄存器：WS2812 配置 */
      if (s_rx_len < 8) return 0;
      {
        uint16_t addr = (uint16_t)((s_rx_buf[2] << 8) | s_rx_buf[3]);
        uint16_t val  = (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]);
        if (addr < MB_WS_REG_BASE || addr >= MB_WS_REG_BASE + MB_WS_REG_COUNT)
        {
          resp_len = mb_build_exception(MB_FC_WRITE_SINGLE_REG, MB_EXC_ILLEGAL_ADDRESS);
        }
        else if (!mb_ws_reg_valid((uint8_t)(addr - MB_WS_REG_BASE), val))
        {
          resp_len = mb_build_exception(MB_FC_WRITE_SINGLE_REG, MB_EXC_ILLEGAL_VALUE);
        }
        else
        {
          s_ws_regs[addr - MB_WS_REG_BASE] = val;
          mb_ws_push_cfg();                          /* 推送给灯效引擎即时生效 */
          s_tx_buf[2] = (uint8_t)(addr >> 8);        /* 响应 = 请求回显 */
          s_tx_buf[3] = (uint8_t)(addr & 0xFFu);
          s_tx_buf[4] = (uint8_t)(val >> 8);
          s_tx_buf[5] = (uint8_t)(val & 0xFFu);
          resp_len = 6;
          {
            uint16_t crc = mb_crc16(s_tx_buf, resp_len);
            s_tx_buf[resp_len++] = (uint8_t)(crc & 0xFFu);
            s_tx_buf[resp_len++] = (uint8_t)(crc >> 8);
          }
        }
      }
      break;

    case MB_FC_WRITE_MULTI_REG:  /* 10 写多寄存器：一帧写灯数/亮度/颜色 */
      if (s_rx_len < 9) return 0;
      {
        uint16_t start = (uint16_t)((s_rx_buf[2] << 8) | s_rx_buf[3]);
        uint16_t qty   = (uint16_t)((s_rx_buf[4] << 8) | s_rx_buf[5]);
        uint8_t  bc    = s_rx_buf[6];
        if (qty == 0u || qty > 123u || bc != (uint8_t)(qty * 2u) ||
            s_rx_len < (uint16_t)(9u + bc))
        {
          resp_len = mb_build_exception(MB_FC_WRITE_MULTI_REG, MB_EXC_ILLEGAL_VALUE);
        }
        else if (start < MB_WS_REG_BASE || start + qty > MB_WS_REG_BASE + MB_WS_REG_COUNT)
        {
          resp_len = mb_build_exception(MB_FC_WRITE_MULTI_REG, MB_EXC_ILLEGAL_ADDRESS);
        }
        else
        {
          uint8_t bad = 0;
          for (uint16_t r = 0u; r < qty; r++)   /* 先全部校验，通过后再写入(原子性) */
          {
            uint16_t v = (uint16_t)((s_rx_buf[7 + r * 2u] << 8) | s_rx_buf[8 + r * 2u]);
            if (!mb_ws_reg_valid((uint8_t)(start - MB_WS_REG_BASE + r), v)) { bad = 1; break; }
          }
          if (bad)
          {
            resp_len = mb_build_exception(MB_FC_WRITE_MULTI_REG, MB_EXC_ILLEGAL_VALUE);
          }
          else
          {
            for (uint16_t r = 0u; r < qty; r++)
            {
              s_ws_regs[start - MB_WS_REG_BASE + r] =
                  (uint16_t)((s_rx_buf[7 + r * 2u] << 8) | s_rx_buf[8 + r * 2u]);
            }
            mb_ws_push_cfg();                          /* 推送给灯效引擎即时生效 */
            s_tx_buf[2] = (uint8_t)(start >> 8);       /* 响应 = 地址+数量回显 */
            s_tx_buf[3] = (uint8_t)(start & 0xFFu);
            s_tx_buf[4] = (uint8_t)(qty >> 8);
            s_tx_buf[5] = (uint8_t)(qty & 0xFFu);
            resp_len = 6;
            {
              uint16_t crc = mb_crc16(s_tx_buf, resp_len);
              s_tx_buf[resp_len++] = (uint8_t)(crc & 0xFFu);
              s_tx_buf[resp_len++] = (uint8_t)(crc >> 8);
            }
          }
        }
      }
      break;

    default:
      resp_len = mb_build_exception(s_rx_buf[1], MB_EXC_ILLEGAL_FUNCTION);
      break;
  }

  if (resp_len > 0)
  {
    /* 诊断：阻塞发送(9600下8字节约8ms)，返回值非 HAL_OK 记录到 s_tx_err */
    if (HAL_UART_Transmit(MB_UART, s_tx_buf, resp_len, 100) != HAL_OK)
    {
      s_tx_err = 1;
    }
  }

  /* 1=收到关闭(断电)继电器指令；0=其它 */
  return close_flag;
}
/* USER CODE END 1 */

/**
  * @brief  UART 接收错误回调(溢出/帧错/噪声错/DMA错...)——"通讯突然挂掉"的恢复点
  * @note   HAL 在 DMA 接收模式下只要出现错误(含溢出 ORE)就走"阻塞性错误"分支：
  *         UART_EndRxTransfer() 先关掉 RXNEIE/PEIE/EIE/IDLEIE，再清 DMAR 并 abort DMA，
  *         最后回调本函数。若这里不重新武装，USART1 将不再产生任何中断
  *         (RXNEIE/IDLEIE/EIE 全为 0)——现象就是"串口突然不通、USART1_IRQHandler
  *         断点不再命中，而主循环心跳照常闪烁"。必须清标志 + 重新武装。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != MB_UART_INST)
  {
    return;
  }

  /* 调试器观察：0x08=溢出ORE  0x02=帧错FE  0x04=噪声NE  0x10=DMA错误 */
  s_mb_rx_err_code = (uint8_t)huart->ErrorCode;
  s_mb_rx_err_cnt++;
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  /* 清错误标志与 IDLE，避免重新武装后立刻又触发 */
  __HAL_UART_CLEAR_FLAG(huart, (UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF |
                                UART_CLEAR_PEF  | UART_CLEAR_IDLEF));

  /* 重新武装接收：这是"通讯自动恢复"的关键 */
  (void)mb_rx_start();
}

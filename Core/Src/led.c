/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    led.c
  * @brief   LED 控制实现
  * @note    LED1/PE9 = 限位指示(任一侧微动/限位触发点亮)
  *          LED2/PE10 = 上位机 Modbus 控制(写线圈地址1)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "di.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private types/functions */
static uint8_t s_led2_state;   /* 上位机控制的 LED2 状态(注意极性处理见 led_out) */
/* USER CODE END Private types/functions */

/* USER CODE BEGIN Private_Function_Prototypes */

/* USER CODE END Private_Function_Prototypes */

/* USER CODE BEGIN Private_Functions */
/* 按极性输出 */
static void led_out(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
#if LED_ACTIVE_HIGH
  HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}
/* USER CODE END Private_Functions */

/**
  * @brief  初始化：两灯全灭
  */
void led_init(void)
{
  s_led2_state = 0;
  led_out(LED1_GPIO_Port, LED1_Pin, 0);
 
}

/**
  * @brief  主循环刷新：LED1 = 限位状态
  * @note   任一侧限位(微动)触发点亮
  */
void led_task(void)
{
  led_out(LED1_GPIO_Port, LED1_Pin, di_get_limit());
}

/**
  * @brief  上位机控制 LED2
  */
void led2_set(uint8_t on)
{
  s_led2_state = on ? 1 : 0;
  
}

/**
  * @brief  上位机控制 LED2 当前状态
  */
uint8_t led2_get(void)
{
  return s_led2_state;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

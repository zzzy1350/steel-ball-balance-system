#ifndef __STATUS_LED_H
#define __STATUS_LED_H

#include "stm32f10x.h"

/**
  * @brief 初始化PA1为推挽输出并默认熄灭低电平有效状态灯。
  * @param 无。
  * @retval 无。
  * @note 只能在上电初始化调用，不能在中断中调用。
  */
void StatusLED_Init(void);

/**
  * @brief 设置状态灯亮灭。
  * @param on 非0表示点亮，0表示熄灭。
  * @retval 无。
  * @note GPIO写操作很短，但本工程统一只在主循环调用；任意非0值均按点亮处理。
  */
void StatusLED_Set(uint8_t on);

/**
  * @brief 翻转状态灯当前软件状态。
  * @param 无。
  * @retval 无。
  * @note 只能在主循环调用，避免与StatusLED_Set并发修改状态。
  */
void StatusLED_Toggle(void);

#endif

#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

/**
  * @brief  初始化PA6上的TIM3_CH1舵机PWM。
  * @param  无。
  * @retval 无。
  * @note   PWM频率固定为50 Hz，一个计数对应1 us；只能在上电初始化时调用，不能在中断中调用。
  */
void PWM_Init(void);

/**
  * @brief  设置TIM3_CH1高电平脉宽。
  * @param  pulse_us 高电平时间，单位us。
  * @retval 无。
  * @note   底层自动限制到1000～2000 us；只能在主循环调用，舵机模块还会执行安全角二次限幅。
  */
void PWM_SetPulseUs(uint16_t pulse_us);

/**
  * @brief  返回当前写入TIM3_CH1的脉宽软件记录值。
  * @param  无。
  * @return 当前脉宽，单位us，范围1000～2000。
  * @note   可在主循环读取；本工程不在中断中调用该接口。
  */
uint16_t PWM_GetPulseUs(void);

#endif

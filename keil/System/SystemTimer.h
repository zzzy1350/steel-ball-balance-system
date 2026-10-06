#ifndef __SYSTEM_TIMER_H
#define __SYSTEM_TIMER_H

#include "stm32f10x.h"

/**
  * @brief  初始化TIM4毫秒系统时基。
  * @param  无。
  * @retval 无。
  * @note   TIM4每1 ms产生一次更新中断，供控制周期、通信超时和任务调度使用。
  *         本函数不能在中断服务程序中调用。
  */
void SystemTimer_Init(void);

/**
  * @brief  获取系统启动后累计的毫秒数。
  * @param  无。
  * @return 32位毫秒计数，约49.7天回绕一次。
  * @note   只读取一个32位volatile变量，可在主循环或中断中调用；不要直接比较绝对大小。
  */
uint32_t SystemTimer_GetMilliseconds(void);

/**
  * @brief  计算从起始时刻到当前时刻经过的毫秒数。
  * @param  now_ms 当前毫秒计数。
  * @param  start_ms 起始毫秒计数。
  * @return 经过的毫秒数，天然兼容32位计数回绕。
  * @note   不访问硬件，可在主循环或中断中调用；时间间隔必须小于一次完整回绕周期。
  */
uint32_t SystemTimer_Elapsed(uint32_t now_ms, uint32_t start_ms);

#endif

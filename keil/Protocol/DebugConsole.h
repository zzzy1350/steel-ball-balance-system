#ifndef __DEBUG_CONSOLE_H
#define __DEBUG_CONSOLE_H

#include "stm32f10x.h"

/**
  * @brief 初始化电脑调参命令行缓存及溢出统计。
  * @param 无。
  * @retval 无。
  * @note 调用前必须初始化USART2；只能在上电初始化调用，不能在中断中调用。
  */
void DebugConsole_Init(void);

/**
  * @brief 非阻塞读取USART2缓存，并在收到换行后解析一条完整调参命令。
  * @param now_ms 当前系统毫秒数，用于目标点命令发送和重发计时。
  * @retval 无。
  * @note 只能在主循环高频调用；半行会保留，超长行会丢弃并增加统计。
  */
void DebugConsole_Process(uint32_t now_ms);

/**
  * @brief 输出目标、坐标、P/I/D、舵机和通信计数等完整状态。
  * @param 无。
  * @retval 无。
  * @note USART2阻塞发送，只能在主循环按需调用，不能在中断中调用。
  */
void DebugConsole_SendStatus(void);

/**
  * @brief 输出一行精简周期状态，供串口助手连续观察。
  * @param 无。
  * @retval 无。
  * @note 建议不高于2 Hz调用；阻塞发送，不能在中断中调用。
  */
void DebugConsole_SendPeriodicStatus(void);

/**
  * @brief 输出所有支持命令及参数格式。
  * @param 无。
  * @retval 无。
  * @note 阻塞发送，只能在主循环或初始化阶段调用，不能在中断中调用。
  */
void DebugConsole_SendHelp(void);

#endif

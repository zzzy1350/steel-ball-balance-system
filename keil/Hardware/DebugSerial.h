#ifndef __DEBUG_SERIAL_H
#define __DEBUG_SERIAL_H

#include "stm32f10x.h"

#define DEBUG_SERIAL_RX_BUFFER_SIZE   128U

/**
  * @brief 初始化USART2、PA2/PA3和128字节接收环形缓冲区。
  * @param 无。
  * @retval 无。
  * @note 固定为115200 8N1；只能在上电初始化调用，不能在中断中调用。
  */
void DebugSerial_Init(void);

/**
  * @brief 从USART2接收环形缓冲区非阻塞读取一个字节。
  * @param byte 输出地址，不能为空。
  * @return 1表示读取成功，0表示缓冲区为空或参数无效。
  * @note 只能由主循环消费，不能在接收中断中再次调用。
  */
uint8_t DebugSerial_ReadByte(uint8_t *byte);

/**
  * @brief 通过USART2阻塞发送一个ASCII字符。
  * @param character 待发送字符。
  * @retval 无。
  * @note 只能在主循环调用，不能在中断中调用。
  */
void DebugSerial_SendChar(char character);

/**
  * @brief 发送以空字符结尾的ASCII字符串。
  * @param text 字符串首地址，为空时直接返回。
  * @retval 无。
  * @note 阻塞发送，只能在主循环调用；字符串必须正确以\0结尾。
  */
void DebugSerial_SendString(const char *text);

/**
  * @brief 不依赖printf，将32位无符号整数转换成十进制ASCII发送。
  * @param value 待发送数值，范围0～4294967295。
  * @retval 无。
  * @note 阻塞发送，只能在主循环调用，不能在中断中调用。
  */
void DebugSerial_SendUInt32(uint32_t value);

/**
  * @brief 不依赖printf，将32位有符号整数转换成十进制ASCII发送。
  * @param value 待发送数值，完整支持INT32最小值。
  * @retval 无。
  * @note 阻塞发送，只能在主循环调用，不能在中断中调用。
  */
void DebugSerial_SendInt32(int32_t value);

/**
  * @brief 发送浮点数，不依赖printf浮点支持。
  * @param value 待发送数值。
  * @param decimals 小数位数，建议0～3。
  * @retval 无。
  * @note 小数位自动限制到3位并四舍五入；只能在主循环调用，输入应为有限数值。
  */
void DebugSerial_SendFloat(float value, uint8_t decimals);

/**
  * @brief 返回USART2环形缓冲区满时丢弃的接收字节累计数。
  * @param 无。
  * @return 溢出字节数，32位计数回绕后从0重新累计。
  * @note 只读访问，可在主循环调用；本工程不在中断中读取。
  */
uint32_t DebugSerial_GetOverflowCount(void);

#endif

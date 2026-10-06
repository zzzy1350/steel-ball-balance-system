#ifndef __K230_SERIAL_H
#define __K230_SERIAL_H

#include "stm32f10x.h"

#define K230_SERIAL_RX_BUFFER_SIZE    128U

/**
  * @brief 初始化USART1、PA9/PA10和128字节接收环形缓冲区。
  * @param 无。
  * @retval 无。
  * @note 固定为115200 8N1；只能在上电初始化调用，不能在中断中调用。
  */
void K230Serial_Init(void);

/**
  * @brief 从USART1环形缓冲区取出一个字节。
  * @param byte 输出地址，成功时写入收到的字节，不能为空。
  * @return 1表示取到数据，0表示缓冲区为空。
  * @note 只能由主循环消费；byte为空时返回0，不会阻塞等待新数据。
  */
uint8_t K230Serial_ReadByte(uint8_t *byte);

/**
  * @brief 通过USART1阻塞发送一个二进制字节。
  * @param byte 待发送的8位数据。
  * @retval 无。
  * @note 只能在主循环调用，不能在中断中调用；函数会等待发送数据寄存器空。
  */
void K230Serial_SendByte(uint8_t byte);

/**
  * @brief 通过USART1连续发送一段二进制数据。
  * @param data 数据首地址，为空时直接返回。
  * @param length 待发送字节数，单位字节；为0时不发送。
  * @retval 无。
  * @note 阻塞发送，只能在主循环调用，不能在中断中调用。
  */
void K230Serial_Send(const uint8_t *data, uint16_t length);

/**
  * @brief 返回USART1环形缓冲区已满时丢弃的接收字节累计数。
  * @param 无。
  * @return 溢出字节数，32位无符号计数回绕后从0重新累计。
  * @note 只读访问，可在主循环调用；本工程不在中断中读取。
  */
uint32_t K230Serial_GetOverflowCount(void);

#endif

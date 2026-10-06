#ifndef __BALANCE_PROTOCOL_H
#define __BALANCE_PROTOCOL_H

#include "stm32f10x.h"

#define BALANCE_PROTOCOL_FRAME_SIZE             14U
#define BALANCE_PROTOCOL_VERSION                0x01U

#define BALANCE_MSG_BALL_STATE                  0x01U
#define BALANCE_MSG_SET_TARGET                  0x10U
#define BALANCE_MSG_TARGET_ACK                  0x11U

#define BALANCE_FLAG_POSITION_VALID             (1U << 0)
#define BALANCE_FLAG_CURRENT_MEASURED           (1U << 1)
#define BALANCE_FLAG_CURRENT_PREDICTED          (1U << 2)
#define BALANCE_FLAG_KALMAN_INITIALIZED         (1U << 3)

/** K230发送的一帧钢球状态。 */
typedef struct
{
    uint8_t sequence;
    uint8_t flags;
    uint16_t x;
    uint16_t y;
    uint16_t confidence_per_mille;
    uint32_t received_ms;
} BalanceBallState;

/** 通信层统计，用于判断接线、波特率、噪声和主循环负载问题。 */
typedef struct
{
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t version_errors;
    uint32_t unknown_messages;
    uint32_t target_ack_frames;
} BalanceProtocolStatistics;

/**
  * @brief 初始化14字节协议解析器、统计量、发送序号和默认目标确认状态。
  * @param 无。
  * @retval 无。
  * @note 调用前必须初始化USART1；只能在上电初始化调用，不能在中断中调用。
  */
void BalanceProtocol_Init(void);

/**
  * @brief 从USART1环形缓冲区取出全部字节并运行协议状态机。
  * @param now_ms 当前系统毫秒数，用于记录钢球数据到达时间。
  * @retval 无。
  * @note 只能在主循环高频调用；不等待半包，能处理噪声、粘包、CRC错误和连续帧头。
  */
void BalanceProtocol_Process(uint32_t now_ms);

/**
  * @brief 取出一份尚未被主控制器消费的最新钢球状态。
  * @param state 输出状态地址，不能为空。
  * @return 1表示返回新状态，0表示没有新状态。
  * @note 只能在主循环调用；若多帧在处理前到达，接口返回最新一帧以降低控制延迟。
  */
uint8_t BalanceProtocol_GetNewBallState(BalanceBallState *state);

/**
  * @brief 设置并立即向K230发送目标点。
  * @param target_x 目标横坐标，单位像素，输入超过319时限制为319。
  * @param target_y 目标纵坐标，单位像素，输入超过319时限制为319。
  * @param now_ms 当前系统毫秒数，用作500 ms重发计时起点。
  * @retval 无。
  * @note 只能在主循环调用；在收到匹配ACK之前，Service函数会每500 ms自动重发。
  */
void BalanceProtocol_SetTarget(uint16_t target_x, uint16_t target_y,
                               uint32_t now_ms);

/**
  * @brief 检查目标ACK状态，并在等待超过500 ms时重发SET_TARGET帧。
  * @param now_ms 当前系统毫秒数。
  * @retval 无。
  * @note 只能在主循环调用；时间差使用无符号减法，兼容毫秒计数回绕。
  */
void BalanceProtocol_Service(uint32_t now_ms);

/**
  * @brief 返回当前目标点是否已经得到内容完全匹配的K230确认。
  * @param 无。
  * @return 1表示已确认，0表示仍等待确认。
  * @note 只读访问，可在主循环调用；设置新目标后自动清零。
  */
uint8_t BalanceProtocol_IsTargetAcknowledged(void);

/**
  * @brief 获取协议有效帧、CRC错误、版本错误和目标ACK等只读统计。
  * @param 无。
  * @return 指向内部静态统计结构的只读指针，始终非空。
  * @note 只能读取，调用者不得修改或长期假设其内容不变；本工程在主循环使用。
  */
const BalanceProtocolStatistics *BalanceProtocol_GetStatistics(void);

/**
  * @brief 计算CRC16/MODBUS，初值0xFFFF、多项式0xA001。
  * @param data 待校验数据首地址；为空时返回初值0xFFFF。
  * @param length 参与计算的字节数，协议帧固定传10。
  * @return 16位CRC值；发送时由调用者按低字节在前写入帧。
  * @note 纯计算函数，但本工程不在中断中运行CRC，以缩短中断响应时间。
  */
uint16_t BalanceProtocol_Crc16Modbus(const uint8_t *data, uint16_t length);

#endif

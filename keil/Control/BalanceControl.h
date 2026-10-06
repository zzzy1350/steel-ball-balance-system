#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#include "stm32f10x.h"
#include "BalanceProtocol.h"

#define BALANCE_DEFAULT_TARGET_X           160U
#define BALANCE_DEFAULT_TARGET_Y           160U
#define BALANCE_VALID_START_FRAMES         5U
#define BALANCE_COMMUNICATION_TIMEOUT_MS   150U

/*
 * 距离自适应倾角参数：误差越小，允许的最大倾角越小。
 * 当横向误差达到100像素时，才允许PID使用完整的±10°输出范围。
 * 如果机械结构需要更积极的远距离回球，可减小FULL_TILT_ERROR；
 * 如果希望整体动作更柔和，可增大FULL_TILT_ERROR或减小MAX_TILT。
 */
#define BALANCE_FULL_TILT_ERROR_PIXELS     100.0f
#define BALANCE_MAX_TILT_DEG               10.0f

typedef enum
{
    BALANCE_STATE_DISABLED = 0,
    BALANCE_STATE_WAITING,
    BALANCE_STATE_ACTIVE,
    BALANCE_STATE_PREDICTING,
    BALANCE_STATE_FAILSAFE
} BalanceState;

/** OLED和调试串口使用的只读状态快照。 */
typedef struct
{
    BalanceState state;
    uint8_t enabled;
    int8_t servo_direction;
    uint16_t target_x;
    uint16_t target_y;
    uint16_t ball_x;
    uint16_t ball_y;
    uint16_t confidence_per_mille;
    uint8_t ball_flags;
    uint8_t consecutive_measured_frames;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float pid_output;
    float distance_tilt_limit;
    float desired_servo_angle;
    float current_servo_angle;
    uint16_t current_servo_pulse_us;
    uint32_t last_ball_ms;
    uint32_t failsafe_count;
    uint32_t communication_timeout_count;
    uint8_t communication_timed_out;
} BalanceControlStatus;

/**
  * @brief 初始化PID、默认目标、方向、帧统计和平衡状态机。
  * @param 无。
  * @retval 无。
  * @note 默认允许闭环，但仍需5帧连续实测；只能在上电初始化调用，不能在中断中调用。
  */
void BalanceControl_Init(void);

/**
  * @brief 把协议层收到的最新BALL_STATE交给平衡状态机并更新连续实测计数。
  * @param ball_state 钢球状态地址，为空时直接返回。
  * @param now_ms 当前系统毫秒数，用于150 ms通信超时判断。
  * @retval 无。
  * @note 只能在主循环调用；坐标、标志和0～319边界会在控制任务中再次校验。
  */
void BalanceControl_OnBallState(const BalanceBallState *ball_state,
                                uint32_t now_ms);

/**
  * @brief 每20 ms运行平衡状态机、失效保护、PID新测量处理和舵机变化率限制。
  * @param now_ms 当前系统毫秒数。
  * @retval 无。
  * @note 只能由主循环20 ms任务调用，不能在中断中调用；无新视觉帧时不会重复计算PID。
  */
void BalanceControl_Run20ms(uint32_t now_ms);

/**
  * @brief 修改本地目标点，清空PID并重新等待5帧连续实测。
  * @param target_x 目标横坐标，单位像素，自动限制到0～319。
  * @param target_y 目标纵坐标，单位像素，自动限制到0～319，仅用于显示。
  * @retval 无。
  * @note 只能在主循环调用；同步到K230由协议层接口另行完成。
  */
void BalanceControl_SetTarget(uint16_t target_x, uint16_t target_y);

/**
  * @brief 修改PID增益，清空积分和微分历史，并重新等待5帧实测。
  * @param kp 比例增益，单位“度/像素”。
  * @param ki 积分增益，单位“度/(像素·秒)”。
  * @param kd 微分增益，单位“度·秒/像素”。
  * @retval 无。
  * @note 只能在主循环调用；有限性和安全范围由电脑命令解析层校验。
  */
void BalanceControl_SetPID(float kp, float ki, float kd);

/**
  * @brief 获取当前PID三项增益。
  * @param kp 比例增益输出地址，可为空。
  * @param ki 积分增益输出地址，可为空。
  * @param kd 微分增益输出地址，可为空。
  * @retval 无。
  * @note 只读接口，可在主循环调用；为空的输出指针会被跳过。
  */
void BalanceControl_GetPID(float *kp, float *ki, float *kd);

/**
  * @brief 启用或禁用闭环控制，并清空PID历史。
  * @param enabled 非0启用，0禁用。
  * @retval 无。
  * @note 只能在主循环调用；重新启用后必须重新累计5帧真实测量。
  */
void BalanceControl_SetEnabled(uint8_t enabled);

/**
  * @brief 设置舵机相对PID输出的机械方向。
  * @param direction 只接受+1或-1，其他值忽略。
  * @retval 无。
  * @note 只能在主循环调用；有效修改会清空PID并重新等待5帧实测。
  */
void BalanceControl_SetServoDirection(int8_t direction);

/**
  * @brief 响应NEUTRAL命令，清空PID目标并让后续控制周期缓慢回到100°。
  * @param 无。
  * @retval 无。
  * @note 只能在主循环调用；控制仍保持启用时，之后重新等待5帧真实测量。
  */
void BalanceControl_RequestNeutral(void);

/**
  * @brief 获取OLED和调试串口所需的完整控制状态快照。
  * @param status 输出结构地址，为空时直接返回。
  * @retval 无。
  * @note 只读复制，只能在主循环调用；舵机角度是命令值而非传感器反馈值。
  */
void BalanceControl_GetStatus(BalanceControlStatus *status);

#endif

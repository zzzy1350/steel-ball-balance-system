#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

#define SERVO_FULL_MIN_ANGLE_DEG       0.0f
#define SERVO_FULL_MAX_ANGLE_DEG       180.0f
#define SERVO_SAFE_MIN_ANGLE_DEG       90.0f
#define SERVO_NEUTRAL_ANGLE_DEG        100.0f
#define SERVO_SAFE_MAX_ANGLE_DEG       110.0f

#define SERVO_FULL_MIN_PULSE_US        1000U
#define SERVO_NEUTRAL_PULSE_US         1556U
#define SERVO_FULL_MAX_PULSE_US        2000U

/**
  * @brief 初始化TIM3舵机PWM，并立即输出100°机械中位命令。
  * @param 无。
  * @retval 无。
  * @note 只能在上电初始化时调用，不能在中断中调用。
  */
void Servo_Init(void);

/**
  * @brief 设置舵机脉宽。
  * @param pulse_us 期望PWM高电平时间，单位us。
  * @retval 无。
  * @note 只能在主循环调用；任何输入都会被限制到90°～110°对应的安全脉宽。
  */
void Servo_SetPulseUs(uint16_t pulse_us);

/**
  * @brief 设置舵机命令角度，并换算成TIM3比较值。
  * @param angle_deg 期望角度，单位度。
  * @retval 无。
  * @note 只能在主循环调用；输入最终限制到90°～110°，不会突破机械安全范围。
  */
void Servo_SetAngle(float angle_deg);

/**
  * @brief 返回软件记录的当前舵机命令角度。
  * @param 无。
  * @return 当前命令角度，单位度，范围90°～110°。
  * @note 该值不是传感器反馈角度；可在主循环读取，本工程不在中断中调用。
  */
float Servo_GetCurrentAngle(void);

/**
  * @brief 返回当前PWM高电平脉宽的软件记录值。
  * @param 无。
  * @return 当前脉宽，单位us，位于安全角对应范围内。
  * @note 可在主循环读取，本工程不在中断中调用。
  */
uint16_t Servo_GetCurrentPulseUs(void);

/**
  * @brief 以指定最大速度向目标角度移动。
  * @param target_angle 目标角度，单位度。
  * @param max_speed_deg_s 最大角速度，单位度/秒。
  * @param dt_s 本次调用对应时间，单位秒。
  * @retval 无。
  * @note 只能在主循环周期任务中调用；负速度会取绝对值，dt小于等于0时不动作。
  */
void Servo_MoveToward(float target_angle, float max_speed_deg_s, float dt_s);

/**
  * @brief 以30°/秒的失效保护速度缓慢回到100°机械中位。
  * @param dt_s 本次调用对应时间，单位秒，正常传入0.02秒。
  * @retval 无。
  * @note 只能在主循环调用；dt小于等于0时不动作。
  */
void Servo_ReturnToNeutral(float dt_s);

#endif

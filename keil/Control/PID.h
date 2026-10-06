#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

/** 一维位置PID控制器及其调试状态。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_measurement;
    float filtered_derivative;
    float p_term;
    float i_term;
    float d_term;
    float output;
    float output_min;
    float output_max;
    float integral_output_limit;
    float derivative_filter_alpha;
    float deadband;
    uint8_t has_previous_measurement;
} PIDController;

/**
  * @brief 初始化一维位置PID控制器，并装载死区、输出限幅、积分限幅和微分滤波默认值。
  * @param pid PID控制器对象地址，不能为空。
  * @param kp 比例增益，单位为“度/像素”。
  * @param ki 积分增益，单位为“度/(像素·秒)”。
  * @param kd 微分增益，单位为“度·秒/像素”。
  * @retval 无。
  * @note 只能在初始化或主循环调用，不能在中断中调用；pid为空时函数直接返回。
  */
void PID_Init(PIDController *pid, float kp, float ki, float kd);

/**
  * @brief 修改PID三项增益，不改变死区、输出限幅和已有历史状态。
  * @param pid PID控制器对象地址，不能为空。
  * @param kp 比例增益，单位为“度/像素”。
  * @param ki 积分增益，单位为“度/(像素·秒)”。
  * @param kd 微分增益，单位为“度·秒/像素”。
  * @retval 无。
  * @note 不能在中断中调用；安全范围由上层命令解析器检查，pid为空时直接返回。
  */
void PID_SetParameters(PIDController *pid, float kp, float ki, float kd);

/**
  * @brief 清除积分、上次测量、微分滤波历史和P/I/D输出，避免状态切换后残留控制量。
  * @param pid PID控制器对象地址，不能为空。
  * @retval 无。
  * @note 只能在主循环调用，不能在中断中调用；pid为空时直接返回。
  */
void PID_Reset(PIDController *pid);

/**
  * @brief 计算一次PID输出。
  * @param error 目标位置减当前位置，单位像素。
  * @param measurement 当前钢球x坐标，单位像素。
  * @param dt_s 两次新视觉测量之间的实际时间，单位秒。
  * @param allow_integral 非0允许更新积分；预测帧应传0。
  * @return 限制在-10°～+10°之间的PID角度增量；控制层还会按钢球距离进一步收紧范围。
  * @note 只能在主循环控制任务中调用，不能在中断中调用；pid为空或dt无效时返回0。
  */
float PID_Compute(PIDController *pid, float error, float measurement,
                  float dt_s, uint8_t allow_integral);

#endif

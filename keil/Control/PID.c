#include "PID.h"

static float PID_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void PID_Init(PIDController *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    /* 本机械结构以100°为中位，只允许左右各摆动10°。 */
    pid->output_min = -10.0f;
    pid->output_max = 10.0f;
    pid->integral_output_limit = 3.0f;
    pid->derivative_filter_alpha = 0.25f;
    pid->deadband = 2.0f;
    PID_SetParameters(pid, kp, ki, kd);
    PID_Reset(pid);
}

void PID_SetParameters(PIDController *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Reset(PIDController *pid)
{
    if (pid == 0)
    {
        return;
    }
    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->output = 0.0f;
    pid->has_previous_measurement = 0U;
}

float PID_Compute(PIDController *pid, float error, float measurement,
                  float dt_s, uint8_t allow_integral)
{
    float effective_error;
    float raw_derivative;
    float candidate_integral;
    float candidate_i_term;
    float unsaturated_output;
    uint8_t drives_further_into_saturation;

    if ((pid == 0) || (dt_s <= 0.000001f))
    {
        return 0.0f;
    }

    effective_error = error;
    if (PID_Abs(effective_error) <= pid->deadband)
    {
        effective_error = 0.0f;
    }

    pid->p_term = pid->kp * effective_error;

    if (pid->has_previous_measurement != 0U)
    {
        raw_derivative = -pid->kd
                       * (measurement - pid->previous_measurement) / dt_s;
        pid->filtered_derivative =
            pid->derivative_filter_alpha * raw_derivative
            + (1.0f - pid->derivative_filter_alpha) * pid->filtered_derivative;
    }
    else
    {
        pid->filtered_derivative = 0.0f;
        pid->has_previous_measurement = 1U;
    }
    pid->previous_measurement = measurement;
    pid->d_term = pid->filtered_derivative;

    candidate_integral = pid->integral;
    if ((allow_integral != 0U) && (effective_error != 0.0f))
    {
        candidate_integral += effective_error * dt_s;
    }

    if (pid->ki > 0.000001f)
    {
        candidate_integral = PID_Clamp(
            candidate_integral,
            -pid->integral_output_limit / pid->ki,
             pid->integral_output_limit / pid->ki);
        candidate_i_term = pid->ki * candidate_integral;
    }
    else
    {
        candidate_integral = 0.0f;
        candidate_i_term = 0.0f;
    }

    unsaturated_output = pid->p_term + candidate_i_term + pid->d_term;
    drives_further_into_saturation = 0U;
    if ((unsaturated_output > pid->output_max) && (effective_error > 0.0f))
    {
        drives_further_into_saturation = 1U;
    }
    else if ((unsaturated_output < pid->output_min) && (effective_error < 0.0f))
    {
        drives_further_into_saturation = 1U;
    }

    /*
     * 当输出已饱和且误差还在把输出推向更深的饱和区时，拒绝本次积分增量。
     * 这样钢球重新回到可控区域后，舵机不会因为积累了大量积分而长时间反向过冲。
     */
    if ((allow_integral != 0U) && (drives_further_into_saturation == 0U))
    {
        pid->integral = candidate_integral;
    }

    pid->i_term = pid->ki * pid->integral;
    pid->i_term = PID_Clamp(pid->i_term,
                            -pid->integral_output_limit,
                             pid->integral_output_limit);
    pid->output = PID_Clamp(pid->p_term + pid->i_term + pid->d_term,
                            pid->output_min, pid->output_max);
    return pid->output;
}

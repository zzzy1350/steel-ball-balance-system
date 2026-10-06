#include "BalanceControl.h"
#include "PID.h"
#include "Servo.h"

#define BALANCE_DEFAULT_KP                   0.070f
#define BALANCE_DEFAULT_KI                   0.030f
#define BALANCE_DEFAULT_KD                   0.050f
#define BALANCE_CONTROL_DT_S                 0.020f
#define BALANCE_SERVO_NORMAL_SPEED_DEG_S     60.0f
#define BALANCE_MIN_MEASUREMENT_DT_S         0.020f
#define BALANCE_MAX_MEASUREMENT_DT_S         0.200f

static PIDController g_pid;
static BalanceState g_state;
static uint8_t g_enabled;
static int8_t g_servo_direction;
static uint16_t g_target_x;
static uint16_t g_target_y;
static BalanceBallState g_ball;
static uint8_t g_has_received_ball_frame;
static uint8_t g_has_new_ball_frame;
static uint8_t g_consecutive_measured_frames;
static uint32_t g_last_ball_ms;
static uint32_t g_last_pid_measurement_ms;
static uint32_t g_failsafe_count;
static uint32_t g_communication_timeout_count;
static uint8_t g_communication_timed_out;
static float g_error;
static float g_distance_tilt_limit;
static float g_desired_servo_angle;

static float Balance_ClampFloat(float value, float minimum, float maximum)
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

static float Balance_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
  * @brief 根据钢球到目标点的横向距离计算当前允许使用的最大倾角。
  * @param error 当前横向误差，单位像素，定义为target_x减ball_x。
  * @return 动态倾角上限，单位度，范围0～BALANCE_MAX_TILT_DEG。
  * @note  误差位于PID死区内时返回0；死区外按距离线性增大，误差达到
  *        BALANCE_FULL_TILT_ERROR_PIXELS后允许使用完整最大倾角。该限制能避免
  *        微分项或积分项在接近中心时仍给出过大的水管倾角。
  */
static float Balance_CalculateDistanceTiltLimit(float error)
{
    float absolute_error;
    float effective_error;
    float usable_error_range;
    float ratio;

    absolute_error = Balance_AbsFloat(error);
    if (absolute_error <= g_pid.deadband)
    {
        return 0.0f;
    }

    effective_error = absolute_error - g_pid.deadband;
    usable_error_range = BALANCE_FULL_TILT_ERROR_PIXELS - g_pid.deadband;
    if (usable_error_range <= 0.001f)
    {
        return BALANCE_MAX_TILT_DEG;
    }

    ratio = Balance_ClampFloat(effective_error / usable_error_range,
                               0.0f, 1.0f);
    return BALANCE_MAX_TILT_DEG * ratio;
}

static uint8_t Balance_IsBallFrameValid(const BalanceBallState *ball_state)
{
    uint8_t has_position;
    uint8_t filter_ready;
    uint8_t source_valid;

    if (ball_state == 0)
    {
        return 0U;
    }

    has_position = (uint8_t)((ball_state->flags & BALANCE_FLAG_POSITION_VALID) != 0U);
    filter_ready = (uint8_t)((ball_state->flags & BALANCE_FLAG_KALMAN_INITIALIZED) != 0U);
    source_valid = (uint8_t)((ball_state->flags
        & (BALANCE_FLAG_CURRENT_MEASURED | BALANCE_FLAG_CURRENT_PREDICTED)) != 0U);

    if ((has_position == 0U) || (filter_ready == 0U) || (source_valid == 0U))
    {
        return 0U;
    }
    if ((ball_state->x > 319U) || (ball_state->y > 319U))
    {
        return 0U;
    }
    return 1U;
}

static void Balance_ResetController(void)
{
    PID_Reset(&g_pid);
    g_error = 0.0f;
    g_distance_tilt_limit = 0.0f;
    g_desired_servo_angle = SERVO_NEUTRAL_ANGLE_DEG;
    g_last_pid_measurement_ms = 0U;
}

/**
  * @brief 进入失效保护状态，并记录本次故障是否由通信超时引起。
  * @param communication_timed_out 非0表示超过150 ms没有收到K230数据；
  *        0表示UART仍有数据，但K230已经报告钢球坐标无效。
  * @note  该函数只能在主循环调用，不能在中断中调用。失效保护总次数只在
  *        状态第一次进入FAILSAFE时增加；通信超时次数由独立锁存逻辑统计。
  */
static void Balance_EnterFailsafe(uint8_t communication_timed_out)
{
    if (g_state != BALANCE_STATE_FAILSAFE)
    {
        g_failsafe_count++;
        Balance_ResetController();
    }
    g_state = BALANCE_STATE_FAILSAFE;
    g_communication_timed_out = (communication_timed_out != 0U) ? 1U : 0U;
    g_consecutive_measured_frames = 0U;
}

void BalanceControl_Init(void)
{
    PID_Init(&g_pid,
             BALANCE_DEFAULT_KP,
             BALANCE_DEFAULT_KI,
             BALANCE_DEFAULT_KD);

    g_state = BALANCE_STATE_WAITING;
    g_enabled = 1U;
    g_servo_direction = 1;
    g_target_x = BALANCE_DEFAULT_TARGET_X;
    g_target_y = BALANCE_DEFAULT_TARGET_Y;
    g_ball.sequence = 0U;
    g_ball.flags = 0U;
    g_ball.x = 0xFFFFU;
    g_ball.y = 0xFFFFU;
    g_ball.confidence_per_mille = 0U;
    g_ball.received_ms = 0U;
    g_has_received_ball_frame = 0U;
    g_has_new_ball_frame = 0U;
    g_consecutive_measured_frames = 0U;
    g_last_ball_ms = 0U;
    g_failsafe_count = 0U;
    g_communication_timeout_count = 0U;
    g_communication_timed_out = 0U;
    Balance_ResetController();
}

void BalanceControl_OnBallState(const BalanceBallState *ball_state,
                                uint32_t now_ms)
{
    uint8_t is_measured;

    if (ball_state == 0)
    {
        return;
    }

    g_ball = *ball_state;
    g_last_ball_ms = now_ms;
    g_has_received_ball_frame = 1U;
    g_has_new_ball_frame = 1U;
    /* 任意完整BALL_STATE到达都说明物理通信已经恢复，哪怕该帧报告丢球。 */
    g_communication_timed_out = 0U;

    if (Balance_IsBallFrameValid(ball_state) == 0U)
    {
        g_consecutive_measured_frames = 0U;
        return;
    }

    is_measured = (uint8_t)((ball_state->flags & BALANCE_FLAG_CURRENT_MEASURED) != 0U);
    if (is_measured != 0U)
    {
        if (g_consecutive_measured_frames < BALANCE_VALID_START_FRAMES)
        {
            g_consecutive_measured_frames++;
        }
    }
    else if ((g_state == BALANCE_STATE_WAITING)
          || (g_state == BALANCE_STATE_FAILSAFE))
    {
        /* 恢复阶段必须连续得到真实测量，预测帧不能算作启动确认帧。 */
        g_consecutive_measured_frames = 0U;
    }
}

void BalanceControl_Run20ms(uint32_t now_ms)
{
    uint8_t ball_valid;
    uint8_t is_predicted;
    uint8_t allow_integral;
    float measurement_dt_s;
    float pid_output;

    if (g_enabled == 0U)
    {
        if (g_state != BALANCE_STATE_DISABLED)
        {
            Balance_ResetController();
            g_state = BALANCE_STATE_DISABLED;
        }
        Servo_ReturnToNeutral(BALANCE_CONTROL_DT_S);
        return;
    }

    if (g_has_received_ball_frame == 0U)
    {
        /*
         * 上电后的前150 ms属于等待K230启动的正常窗口，不立即记作通信故障。
         * 超过窗口仍没有收到第一帧，才进入UART超时失效保护。
         */
        if (now_ms > BALANCE_COMMUNICATION_TIMEOUT_MS)
        {
            if (g_communication_timed_out == 0U)
            {
                g_communication_timeout_count++;
            }
            Balance_EnterFailsafe(1U);
        }
        else
        {
            g_state = BALANCE_STATE_WAITING;
        }
        Servo_ReturnToNeutral(BALANCE_CONTROL_DT_S);
        return;
    }

    if ((uint32_t)(now_ms - g_last_ball_ms) > BALANCE_COMMUNICATION_TIMEOUT_MS)
    {
        if (g_communication_timed_out == 0U)
        {
            /* 每次连续掉线只计数一次，收到下一帧后才允许再次计数。 */
            g_communication_timeout_count++;
        }
        Balance_EnterFailsafe(1U);
        Servo_ReturnToNeutral(BALANCE_CONTROL_DT_S);
        return;
    }

    ball_valid = Balance_IsBallFrameValid(&g_ball);
    if (ball_valid == 0U)
    {
        Balance_EnterFailsafe(0U);
        Servo_ReturnToNeutral(BALANCE_CONTROL_DT_S);
        return;
    }

    if (g_consecutive_measured_frames < BALANCE_VALID_START_FRAMES)
    {
        if (g_state != BALANCE_STATE_WAITING)
        {
            Balance_ResetController();
        }
        g_state = BALANCE_STATE_WAITING;
        Servo_ReturnToNeutral(BALANCE_CONTROL_DT_S);
        return;
    }

    is_predicted = (uint8_t)((g_ball.flags & BALANCE_FLAG_CURRENT_PREDICTED) != 0U);
    allow_integral = (uint8_t)(is_predicted == 0U);
    g_state = (is_predicted != 0U)
            ? BALANCE_STATE_PREDICTING
            : BALANCE_STATE_ACTIVE;

    /*
     * PID只在K230送来一帧新坐标时重新计算。控制循环其余时间只负责让舵机
     * 按限速逐步靠近目标角，避免把10～30 Hz视觉坐标错误地当成50 Hz新测量。
     */
    if (g_has_new_ball_frame != 0U)
    {
        if (g_last_pid_measurement_ms == 0U)
        {
            measurement_dt_s = BALANCE_CONTROL_DT_S;
        }
        else
        {
            measurement_dt_s = (float)((uint32_t)(now_ms - g_last_pid_measurement_ms))
                             / 1000.0f;
            measurement_dt_s = Balance_ClampFloat(
                measurement_dt_s,
                BALANCE_MIN_MEASUREMENT_DT_S,
                BALANCE_MAX_MEASUREMENT_DT_S);
        }
        g_last_pid_measurement_ms = now_ms;

        g_error = (float)g_target_x - (float)g_ball.x;

        /*
         * 在计算PID前，根据当前位置动态收紧PID输出上下限。这样PID内部的
         * 条件积分防饱和也会使用同一个动态边界，而不是计算后再简单截断。
         * 例如默认参数下：误差约10/50/100像素时，最大倾角约为
         * 1.2°/7.3°/15°；越接近中心，水管动作越柔和。
         */
        g_distance_tilt_limit = Balance_CalculateDistanceTiltLimit(g_error);
        g_pid.output_min = -g_distance_tilt_limit;
        g_pid.output_max = g_distance_tilt_limit;

        pid_output = PID_Compute(&g_pid, g_error, (float)g_ball.x,
                                 measurement_dt_s, allow_integral);
        g_desired_servo_angle = SERVO_NEUTRAL_ANGLE_DEG
                               + (float)g_servo_direction * pid_output;
        g_desired_servo_angle = Balance_ClampFloat(
            g_desired_servo_angle,
            SERVO_SAFE_MIN_ANGLE_DEG,
            SERVO_SAFE_MAX_ANGLE_DEG);
        g_has_new_ball_frame = 0U;
    }

    Servo_MoveToward(g_desired_servo_angle,
                     BALANCE_SERVO_NORMAL_SPEED_DEG_S,
                     BALANCE_CONTROL_DT_S);
}

void BalanceControl_SetTarget(uint16_t target_x, uint16_t target_y)
{
    g_target_x = (target_x > 319U) ? 319U : target_x;
    g_target_y = (target_y > 319U) ? 319U : target_y;
    g_consecutive_measured_frames = 0U;
    Balance_ResetController();
    g_state = (g_enabled != 0U) ? BALANCE_STATE_WAITING : BALANCE_STATE_DISABLED;
}

void BalanceControl_SetPID(float kp, float ki, float kd)
{
    PID_SetParameters(&g_pid, kp, ki, kd);
    Balance_ResetController();
    g_consecutive_measured_frames = 0U;
    g_state = (g_enabled != 0U) ? BALANCE_STATE_WAITING : BALANCE_STATE_DISABLED;
}

void BalanceControl_GetPID(float *kp, float *ki, float *kd)
{
    if (kp != 0)
    {
        *kp = g_pid.kp;
    }
    if (ki != 0)
    {
        *ki = g_pid.ki;
    }
    if (kd != 0)
    {
        *kd = g_pid.kd;
    }
}

void BalanceControl_SetEnabled(uint8_t enabled)
{
    g_enabled = (enabled != 0U) ? 1U : 0U;
    g_consecutive_measured_frames = 0U;
    Balance_ResetController();
    g_state = (g_enabled != 0U) ? BALANCE_STATE_WAITING : BALANCE_STATE_DISABLED;
}

void BalanceControl_SetServoDirection(int8_t direction)
{
    if ((direction == 1) || (direction == -1))
    {
        g_servo_direction = direction;
        Balance_ResetController();
        g_consecutive_measured_frames = 0U;
        g_state = (g_enabled != 0U) ? BALANCE_STATE_WAITING : BALANCE_STATE_DISABLED;
    }
}

void BalanceControl_RequestNeutral(void)
{
    Balance_ResetController();
    g_consecutive_measured_frames = 0U;
    g_state = (g_enabled != 0U) ? BALANCE_STATE_WAITING : BALANCE_STATE_DISABLED;
}

void BalanceControl_GetStatus(BalanceControlStatus *status)
{
    if (status == 0)
    {
        return;
    }

    status->state = g_state;
    status->enabled = g_enabled;
    status->servo_direction = g_servo_direction;
    status->target_x = g_target_x;
    status->target_y = g_target_y;
    status->ball_x = g_ball.x;
    status->ball_y = g_ball.y;
    status->confidence_per_mille = g_ball.confidence_per_mille;
    status->ball_flags = g_ball.flags;
    status->consecutive_measured_frames = g_consecutive_measured_frames;
    status->error = g_error;
    status->p_term = g_pid.p_term;
    status->i_term = g_pid.i_term;
    status->d_term = g_pid.d_term;
    status->pid_output = g_pid.output;
    status->distance_tilt_limit = g_distance_tilt_limit;
    status->desired_servo_angle = g_desired_servo_angle;
    status->current_servo_angle = Servo_GetCurrentAngle();
    status->current_servo_pulse_us = Servo_GetCurrentPulseUs();
    status->last_ball_ms = g_last_ball_ms;
    status->failsafe_count = g_failsafe_count;
    status->communication_timeout_count = g_communication_timeout_count;
    status->communication_timed_out = g_communication_timed_out;
}

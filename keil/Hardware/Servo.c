#include "Servo.h"
#include "PWM.h"

#define SERVO_FAILSAFE_SPEED_DEG_S     30.0f

static float g_servo_current_angle = SERVO_NEUTRAL_ANGLE_DEG;

static float Servo_ClampAngle(float angle_deg)
{
    if (angle_deg < SERVO_SAFE_MIN_ANGLE_DEG)
    {
        return SERVO_SAFE_MIN_ANGLE_DEG;
    }
    if (angle_deg > SERVO_SAFE_MAX_ANGLE_DEG)
    {
        return SERVO_SAFE_MAX_ANGLE_DEG;
    }
    return angle_deg;
}

static uint16_t Servo_AngleToPulseUs(float angle_deg)
{
    float pulse;

    angle_deg = Servo_ClampAngle(angle_deg);
    pulse = (float)SERVO_FULL_MIN_PULSE_US
          + angle_deg * (float)(SERVO_FULL_MAX_PULSE_US - SERVO_FULL_MIN_PULSE_US)
          / (SERVO_FULL_MAX_ANGLE_DEG - SERVO_FULL_MIN_ANGLE_DEG);
    return (uint16_t)(pulse + 0.5f);
}

static float Servo_PulseUsToAngle(uint16_t pulse_us)
{
    return ((float)(pulse_us - SERVO_FULL_MIN_PULSE_US)
            * (SERVO_FULL_MAX_ANGLE_DEG - SERVO_FULL_MIN_ANGLE_DEG)
            / (float)(SERVO_FULL_MAX_PULSE_US - SERVO_FULL_MIN_PULSE_US));
}

void Servo_Init(void)
{
    PWM_Init();

    /*
     * 机械实测水管水平位置对应舵机100°。这里通过统一角度接口自动换算PWM，
     * 避免仍写入1500 us（90°）而导致软件记录角度与实际输出不一致。
     */
    Servo_SetAngle(SERVO_NEUTRAL_ANGLE_DEG);
}

void Servo_SetPulseUs(uint16_t pulse_us)
{
    uint16_t safe_min_pulse;
    uint16_t safe_max_pulse;

    safe_min_pulse = Servo_AngleToPulseUs(SERVO_SAFE_MIN_ANGLE_DEG);
    safe_max_pulse = Servo_AngleToPulseUs(SERVO_SAFE_MAX_ANGLE_DEG);

    if (pulse_us < safe_min_pulse)
    {
        pulse_us = safe_min_pulse;
    }
    else if (pulse_us > safe_max_pulse)
    {
        pulse_us = safe_max_pulse;
    }

    g_servo_current_angle = Servo_PulseUsToAngle(pulse_us);
    PWM_SetPulseUs(pulse_us);
}

void Servo_SetAngle(float angle_deg)
{
    angle_deg = Servo_ClampAngle(angle_deg);
    g_servo_current_angle = angle_deg;
    PWM_SetPulseUs(Servo_AngleToPulseUs(angle_deg));
}

float Servo_GetCurrentAngle(void)
{
    return g_servo_current_angle;
}

uint16_t Servo_GetCurrentPulseUs(void)
{
    return PWM_GetPulseUs();
}

void Servo_MoveToward(float target_angle, float max_speed_deg_s, float dt_s)
{
    float maximum_step;
    float difference;

    target_angle = Servo_ClampAngle(target_angle);
    if (max_speed_deg_s < 0.0f)
    {
        max_speed_deg_s = -max_speed_deg_s;
    }
    if (dt_s <= 0.0f)
    {
        return;
    }

    maximum_step = max_speed_deg_s * dt_s;
    difference = target_angle - g_servo_current_angle;

    if (difference > maximum_step)
    {
        difference = maximum_step;
    }
    else if (difference < -maximum_step)
    {
        difference = -maximum_step;
    }

    Servo_SetAngle(g_servo_current_angle + difference);
}

void Servo_ReturnToNeutral(float dt_s)
{
    Servo_MoveToward(SERVO_NEUTRAL_ANGLE_DEG,
                     SERVO_FAILSAFE_SPEED_DEG_S, dt_s);
}

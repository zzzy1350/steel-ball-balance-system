#include "PWM.h"

#define PWM_MIN_PULSE_US    1000U
#define PWM_MAX_PULSE_US    2000U
#define PWM_PERIOD_US       20000U

static uint16_t g_pwm_pulse_us = 1500U;

void PWM_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    TIM_TimeBaseInitTypeDef timer_init;
    TIM_OCInitTypeDef output_compare_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* PA6是TIM3_CH1默认复用引脚，配置为50 MHz复用推挽输出。 */
    gpio_init.GPIO_Pin = GPIO_Pin_6;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init);

    /* 72 MHz / 72 = 1 MHz，因此计数器每增加1对应1 us。 */
    TIM_TimeBaseStructInit(&timer_init);
    timer_init.TIM_Prescaler = 72U - 1U;
    timer_init.TIM_Period = PWM_PERIOD_US - 1U;
    timer_init.TIM_CounterMode = TIM_CounterMode_Up;
    timer_init.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &timer_init);

    /* PWM模式1：计数值小于CCR1时输出有效高电平。 */
    TIM_OCStructInit(&output_compare_init);
    output_compare_init.TIM_OCMode = TIM_OCMode_PWM1;
    output_compare_init.TIM_OutputState = TIM_OutputState_Enable;
    output_compare_init.TIM_OCPolarity = TIM_OCPolarity_High;
    output_compare_init.TIM_Pulse = 1500U;
    TIM_OC1Init(TIM3, &output_compare_init);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    g_pwm_pulse_us = 1500U;
    TIM_Cmd(TIM3, ENABLE);
}

void PWM_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < PWM_MIN_PULSE_US)
    {
        pulse_us = PWM_MIN_PULSE_US;
    }
    else if (pulse_us > PWM_MAX_PULSE_US)
    {
        pulse_us = PWM_MAX_PULSE_US;
    }

    g_pwm_pulse_us = pulse_us;
    TIM_SetCompare1(TIM3, pulse_us);
}

uint16_t PWM_GetPulseUs(void)
{
    return g_pwm_pulse_us;
}

#include "StatusLED.h"

void StatusLED_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio_init.GPIO_Pin = GPIO_Pin_1;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio_init);

    /* 现有最小系统工程的PA1 LED为低电平点亮，上电默认先熄灭。 */
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
}

void StatusLED_Set(uint8_t on)
{
    if (on != 0U)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    }
    else
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
    }
}

void StatusLED_Toggle(void)
{
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    }
}

#include "SystemTimer.h"

/*
 * 该变量只在TIM4中断中写入，在主循环中读取，因此必须声明为volatile，
 * 防止编译器把主循环中的读取优化成一个永远不更新的寄存器副本。
 */
static volatile uint32_t g_system_milliseconds = 0U;

void SystemTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef timer_init;
    NVIC_InitTypeDef nvic_init;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /*
     * TIM4位于APB1。系统时钟72 MHz、APB1分频为2时，定时器时钟仍为72 MHz。
     * 72 MHz / 7200 / 10 = 1000 Hz，因此更新周期正好为1 ms。
     */
    TIM_TimeBaseStructInit(&timer_init);
    timer_init.TIM_Prescaler = 7200U - 1U;
    timer_init.TIM_Period = 10U - 1U;
    timer_init.TIM_CounterMode = TIM_CounterMode_Up;
    timer_init.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &timer_init);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    /*
     * 串口接收比系统节拍更容易丢数据，因此TIM4中断优先级低于USART1/USART2。
     */
    nvic_init.NVIC_IRQChannel = TIM4_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic_init.NVIC_IRQChannelSubPriority = 0U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    g_system_milliseconds = 0U;
    TIM_Cmd(TIM4, ENABLE);
}

uint32_t SystemTimer_GetMilliseconds(void)
{
    return g_system_milliseconds;
}

uint32_t SystemTimer_Elapsed(uint32_t now_ms, uint32_t start_ms)
{
    return (uint32_t)(now_ms - start_ms);
}

/**
  * @brief TIM4更新中断，只维护毫秒计数。
  * @note  中断中不执行PID、串口解析、OLED刷新或任何阻塞操作。
  */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        g_system_milliseconds++;
    }
}

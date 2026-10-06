#include "K230Serial.h"

static volatile uint8_t g_k230_rx_buffer[K230_SERIAL_RX_BUFFER_SIZE];
static volatile uint16_t g_k230_rx_head = 0U;
static volatile uint16_t g_k230_rx_tail = 0U;
static volatile uint32_t g_k230_overflow_count = 0U;

void K230Serial_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    gpio_init.GPIO_Pin = GPIO_Pin_9;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init);

    gpio_init.GPIO_Pin = GPIO_Pin_10;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio_init);

    USART_StructInit(&usart_init);
    usart_init.USART_BaudRate = 115200U;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart_init);

    g_k230_rx_head = 0U;
    g_k230_rx_tail = 0U;
    g_k230_overflow_count = 0U;

    USART_ClearFlag(USART1, USART_FLAG_RXNE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic_init.NVIC_IRQChannel = USART1_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority = 0U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    USART_Cmd(USART1, ENABLE);
}

uint8_t K230Serial_ReadByte(uint8_t *byte)
{
    if ((byte == 0) || (g_k230_rx_tail == g_k230_rx_head))
    {
        return 0U;
    }

    *byte = g_k230_rx_buffer[g_k230_rx_tail];
    g_k230_rx_tail = (uint16_t)((g_k230_rx_tail + 1U) % K230_SERIAL_RX_BUFFER_SIZE);
    return 1U;
}

void K230Serial_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, byte);
}

void K230Serial_Send(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == 0)
    {
        return;
    }

    for (index = 0U; index < length; index++)
    {
        K230Serial_SendByte(data[index]);
    }
}

uint32_t K230Serial_GetOverflowCount(void)
{
    return g_k230_overflow_count;
}

/**
  * @brief USART1接收中断。
  * @note  中断只搬运字节，不进行CRC和协议解析，以保证中断足够短。
  */
void USART1_IRQHandler(void)
{
    uint16_t next_head;
    uint8_t byte;
    volatile uint16_t status;

    status = USART1->SR;
    if ((status & USART_SR_RXNE) != 0U)
    {
        byte = (uint8_t)USART_ReceiveData(USART1);
        next_head = (uint16_t)((g_k230_rx_head + 1U) % K230_SERIAL_RX_BUFFER_SIZE);
        if (next_head != g_k230_rx_tail)
        {
            g_k230_rx_buffer[g_k230_rx_head] = byte;
            g_k230_rx_head = next_head;
        }
        else
        {
            g_k230_overflow_count++;
        }
    }
    else if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        /* 读取DR可以清除ORE/NE/FE/PE错误状态，错误字节直接丢弃。 */
        byte = (uint8_t)USART1->DR;
        (void)byte;
    }
}

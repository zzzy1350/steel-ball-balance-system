#include "DebugSerial.h"

static volatile uint8_t g_debug_rx_buffer[DEBUG_SERIAL_RX_BUFFER_SIZE];
static volatile uint16_t g_debug_rx_head = 0U;
static volatile uint16_t g_debug_rx_tail = 0U;
static volatile uint32_t g_debug_overflow_count = 0U;

void DebugSerial_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2原来用于示例LED2，本工程改为USART2_TX，不再调用旧LED2接口。 */
    gpio_init.GPIO_Pin = GPIO_Pin_2;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init);

    gpio_init.GPIO_Pin = GPIO_Pin_3;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio_init);

    USART_StructInit(&usart_init);
    usart_init.USART_BaudRate = 115200U;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart_init);

    g_debug_rx_head = 0U;
    g_debug_rx_tail = 0U;
    g_debug_overflow_count = 0U;

    USART_ClearFlag(USART2, USART_FLAG_RXNE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic_init.NVIC_IRQChannel = USART2_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic_init.NVIC_IRQChannelSubPriority = 0U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    USART_Cmd(USART2, ENABLE);
}

uint8_t DebugSerial_ReadByte(uint8_t *byte)
{
    if ((byte == 0) || (g_debug_rx_tail == g_debug_rx_head))
    {
        return 0U;
    }

    *byte = g_debug_rx_buffer[g_debug_rx_tail];
    g_debug_rx_tail = (uint16_t)((g_debug_rx_tail + 1U) % DEBUG_SERIAL_RX_BUFFER_SIZE);
    return 1U;
}

void DebugSerial_SendChar(char character)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART2, (uint8_t)character);
}

void DebugSerial_SendString(const char *text)
{
    if (text == 0)
    {
        return;
    }

    while (*text != '\0')
    {
        DebugSerial_SendChar(*text);
        text++;
    }
}

void DebugSerial_SendUInt32(uint32_t value)
{
    char buffer[10];
    uint8_t length;

    if (value == 0U)
    {
        DebugSerial_SendChar('0');
        return;
    }

    length = 0U;
    while ((value > 0U) && (length < sizeof(buffer)))
    {
        buffer[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (length > 0U)
    {
        DebugSerial_SendChar(buffer[--length]);
    }
}

void DebugSerial_SendInt32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        DebugSerial_SendChar('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }
    DebugSerial_SendUInt32(magnitude);
}

void DebugSerial_SendFloat(float value, uint8_t decimals)
{
    uint32_t multiplier;
    uint32_t scaled;
    uint32_t integer_part;
    uint32_t fraction_part;
    uint8_t index;

    if (decimals > 3U)
    {
        decimals = 3U;
    }

    if (value < 0.0f)
    {
        DebugSerial_SendChar('-');
        value = -value;
    }

    multiplier = 1U;
    for (index = 0U; index < decimals; index++)
    {
        multiplier *= 10U;
    }

    scaled = (uint32_t)(value * (float)multiplier + 0.5f);
    integer_part = scaled / multiplier;
    fraction_part = scaled % multiplier;
    DebugSerial_SendUInt32(integer_part);

    if (decimals > 0U)
    {
        DebugSerial_SendChar('.');
        multiplier /= 10U;
        while (multiplier > 0U)
        {
            DebugSerial_SendChar((char)('0' + ((fraction_part / multiplier) % 10U)));
            multiplier /= 10U;
        }
    }
}

uint32_t DebugSerial_GetOverflowCount(void)
{
    return g_debug_overflow_count;
}

void USART2_IRQHandler(void)
{
    uint16_t next_head;
    uint8_t byte;
    volatile uint16_t status;

    status = USART2->SR;
    if ((status & USART_SR_RXNE) != 0U)
    {
        byte = (uint8_t)USART_ReceiveData(USART2);
        next_head = (uint16_t)((g_debug_rx_head + 1U) % DEBUG_SERIAL_RX_BUFFER_SIZE);
        if (next_head != g_debug_rx_tail)
        {
            g_debug_rx_buffer[g_debug_rx_head] = byte;
            g_debug_rx_head = next_head;
        }
        else
        {
            g_debug_overflow_count++;
        }
    }
    else if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        byte = (uint8_t)USART2->DR;
        (void)byte;
    }
}

#include "BalanceProtocol.h"
#include "K230Serial.h"

#define PROTOCOL_SOF1                   0xAAU
#define PROTOCOL_SOF2                   0x55U
#define TARGET_RESEND_PERIOD_MS         500U

static uint8_t g_rx_frame[BALANCE_PROTOCOL_FRAME_SIZE];
static uint8_t g_rx_index = 0U;
static uint8_t g_tx_sequence = 0U;

static BalanceBallState g_latest_ball_state;
static uint8_t g_has_new_ball_state = 0U;

static uint16_t g_target_x = 160U;
static uint16_t g_target_y = 160U;
static uint8_t g_target_acknowledged = 0U;
static uint32_t g_last_target_send_ms = 0U;

static BalanceProtocolStatistics g_statistics;

static void Protocol_PutU16LE(uint8_t *buffer, uint8_t index, uint16_t value)
{
    buffer[index] = (uint8_t)(value & 0x00FFU);
    buffer[index + 1U] = (uint8_t)((value >> 8U) & 0x00FFU);
}

static uint16_t Protocol_GetU16LE(const uint8_t *buffer, uint8_t index)
{
    return (uint16_t)buffer[index]
         | ((uint16_t)buffer[index + 1U] << 8U);
}

uint16_t BalanceProtocol_Crc16Modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc;
    uint16_t index;
    uint8_t bit;

    crc = 0xFFFFU;
    if (data == 0)
    {
        return crc;
    }

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static void Protocol_SendFrame(uint8_t message_type, uint8_t flags,
                               uint16_t x, uint16_t y, uint16_t auxiliary)
{
    uint8_t frame[BALANCE_PROTOCOL_FRAME_SIZE];
    uint16_t crc;

    frame[0] = PROTOCOL_SOF1;
    frame[1] = PROTOCOL_SOF2;
    frame[2] = BALANCE_PROTOCOL_VERSION;
    frame[3] = message_type;
    frame[4] = g_tx_sequence++;
    frame[5] = flags;
    Protocol_PutU16LE(frame, 6U, x);
    Protocol_PutU16LE(frame, 8U, y);
    Protocol_PutU16LE(frame, 10U, auxiliary);

    /* CRC只覆盖版本、类型、序号、标志和6字节数据，即frame[2]～frame[11]。 */
    crc = BalanceProtocol_Crc16Modbus(&frame[2], 10U);
    Protocol_PutU16LE(frame, 12U, crc);
    K230Serial_Send(frame, BALANCE_PROTOCOL_FRAME_SIZE);
}

static void Protocol_ProcessCompleteFrame(const uint8_t *frame, uint32_t now_ms)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t message_type;
    uint16_t x;
    uint16_t y;

    received_crc = Protocol_GetU16LE(frame, 12U);
    calculated_crc = BalanceProtocol_Crc16Modbus(&frame[2], 10U);
    if (received_crc != calculated_crc)
    {
        g_statistics.crc_errors++;
        return;
    }

    if (frame[2] != BALANCE_PROTOCOL_VERSION)
    {
        g_statistics.version_errors++;
        return;
    }

    g_statistics.valid_frames++;
    message_type = frame[3];
    x = Protocol_GetU16LE(frame, 6U);
    y = Protocol_GetU16LE(frame, 8U);

    if (message_type == BALANCE_MSG_BALL_STATE)
    {
        g_latest_ball_state.sequence = frame[4];
        g_latest_ball_state.flags = frame[5];
        g_latest_ball_state.x = x;
        g_latest_ball_state.y = y;
        g_latest_ball_state.confidence_per_mille = Protocol_GetU16LE(frame, 10U);
        g_latest_ball_state.received_ms = now_ms;
        g_has_new_ball_state = 1U;
    }
    else if (message_type == BALANCE_MSG_TARGET_ACK)
    {
        g_statistics.target_ack_frames++;
        if ((x == g_target_x) && (y == g_target_y))
        {
            g_target_acknowledged = 1U;
        }
    }
    else
    {
        g_statistics.unknown_messages++;
    }
}

void BalanceProtocol_Init(void)
{
    uint8_t index;

    g_rx_index = 0U;
    g_tx_sequence = 0U;
    g_has_new_ball_state = 0U;
    g_target_x = 160U;
    g_target_y = 160U;
    g_target_acknowledged = 0U;
    g_last_target_send_ms = 0U;

    for (index = 0U; index < BALANCE_PROTOCOL_FRAME_SIZE; index++)
    {
        g_rx_frame[index] = 0U;
    }

    g_statistics.valid_frames = 0U;
    g_statistics.crc_errors = 0U;
    g_statistics.version_errors = 0U;
    g_statistics.unknown_messages = 0U;
    g_statistics.target_ack_frames = 0U;
}

void BalanceProtocol_Process(uint32_t now_ms)
{
    uint8_t byte;

    while (K230Serial_ReadByte(&byte) != 0U)
    {
        if (g_rx_index == 0U)
        {
            if (byte == PROTOCOL_SOF1)
            {
                g_rx_frame[0] = byte;
                g_rx_index = 1U;
            }
            continue;
        }

        if (g_rx_index == 1U)
        {
            if (byte == PROTOCOL_SOF2)
            {
                g_rx_frame[1] = byte;
                g_rx_index = 2U;
            }
            else if (byte == PROTOCOL_SOF1)
            {
                /* 连续0xAA时，第二个0xAA仍可作为新帧第一个字节。 */
                g_rx_frame[0] = byte;
                g_rx_index = 1U;
            }
            else
            {
                g_rx_index = 0U;
            }
            continue;
        }

        g_rx_frame[g_rx_index++] = byte;
        if (g_rx_index >= BALANCE_PROTOCOL_FRAME_SIZE)
        {
            Protocol_ProcessCompleteFrame(g_rx_frame, now_ms);

            /* 若最后一个字节恰好是0xAA，保留为下一帧候选帧头。 */
            if (byte == PROTOCOL_SOF1)
            {
                g_rx_frame[0] = byte;
                g_rx_index = 1U;
            }
            else
            {
                g_rx_index = 0U;
            }
        }
    }
}

uint8_t BalanceProtocol_GetNewBallState(BalanceBallState *state)
{
    if ((state == 0) || (g_has_new_ball_state == 0U))
    {
        return 0U;
    }

    *state = g_latest_ball_state;
    g_has_new_ball_state = 0U;
    return 1U;
}

void BalanceProtocol_SetTarget(uint16_t target_x, uint16_t target_y,
                               uint32_t now_ms)
{
    if (target_x > 319U)
    {
        target_x = 319U;
    }
    if (target_y > 319U)
    {
        target_y = 319U;
    }

    g_target_x = target_x;
    g_target_y = target_y;
    g_target_acknowledged = 0U;
    g_last_target_send_ms = now_ms;
    Protocol_SendFrame(BALANCE_MSG_SET_TARGET, 0U,
                       g_target_x, g_target_y, 0U);
}

void BalanceProtocol_Service(uint32_t now_ms)
{
    if ((g_target_acknowledged == 0U)
        && ((uint32_t)(now_ms - g_last_target_send_ms) >= TARGET_RESEND_PERIOD_MS))
    {
        g_last_target_send_ms = now_ms;
        Protocol_SendFrame(BALANCE_MSG_SET_TARGET, 0U,
                           g_target_x, g_target_y, 0U);
    }
}

uint8_t BalanceProtocol_IsTargetAcknowledged(void)
{
    return g_target_acknowledged;
}

const BalanceProtocolStatistics *BalanceProtocol_GetStatistics(void)
{
    return &g_statistics;
}

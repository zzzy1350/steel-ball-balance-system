#include "DebugConsole.h"
#include "DebugSerial.h"
#include "K230Serial.h"
#include "BalanceProtocol.h"
#include "BalanceControl.h"

#define DEBUG_LINE_BUFFER_SIZE       96U
#define DEBUG_MAX_TOKENS             5U

static char g_line_buffer[DEBUG_LINE_BUFFER_SIZE];
static uint8_t g_line_length = 0U;
static uint32_t g_line_overflow_count = 0U;

static char Debug_ToUpper(char character)
{
    if ((character >= 'a') && (character <= 'z'))
    {
        return (char)(character - 'a' + 'A');
    }
    return character;
}

static uint8_t Debug_StringEqualsIgnoreCase(const char *left, const char *right)
{
    if ((left == 0) || (right == 0))
    {
        return 0U;
    }

    while ((*left != '\0') && (*right != '\0'))
    {
        if (Debug_ToUpper(*left) != Debug_ToUpper(*right))
        {
            return 0U;
        }
        left++;
        right++;
    }
    return (uint8_t)((*left == '\0') && (*right == '\0'));
}

static uint8_t Debug_SplitTokens(char *line, char **tokens, uint8_t maximum_tokens)
{
    uint8_t count;
    uint8_t in_token;

    count = 0U;
    in_token = 0U;
    while (*line != '\0')
    {
        if ((*line == ' ') || (*line == '\t'))
        {
            *line = '\0';
            in_token = 0U;
        }
        else if (in_token == 0U)
        {
            if (count >= maximum_tokens)
            {
                return count;
            }
            tokens[count++] = line;
            in_token = 1U;
        }
        line++;
    }
    return count;
}

static uint8_t Debug_ParseInt32(const char *text, int32_t *value)
{
    uint32_t magnitude;
    uint32_t limit;
    uint32_t digit;
    uint8_t negative;
    uint8_t has_digit;

    if ((text == 0) || (value == 0))
    {
        return 0U;
    }

    magnitude = 0U;
    negative = 0U;
    has_digit = 0U;
    if (*text == '-')
    {
        negative = 1U;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    limit = (negative != 0U) ? 2147483648UL : 2147483647UL;
    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9'))
        {
            return 0U;
        }
        has_digit = 1U;
        digit = (uint32_t)(*text - '0');
        /* 在乘10以前检查边界，避免恶意或误输入的超长数字触发有符号溢出。 */
        if (magnitude > (limit - digit) / 10U)
        {
            return 0U;
        }
        magnitude = magnitude * 10U + digit;
        text++;
    }

    if (has_digit == 0U)
    {
        return 0U;
    }
    if (negative != 0U)
    {
        if (magnitude == 2147483648UL)
        {
            *value = (-2147483647L - 1L);
        }
        else
        {
            *value = -(int32_t)magnitude;
        }
    }
    else
    {
        *value = (int32_t)magnitude;
    }
    return 1U;
}

static uint8_t Debug_ParseFloat(const char *text, float *value)
{
    float result;
    float fraction_scale;
    float sign;
    uint8_t has_digit;
    uint8_t after_decimal;

    if ((text == 0) || (value == 0))
    {
        return 0U;
    }

    result = 0.0f;
    fraction_scale = 0.1f;
    sign = 1.0f;
    has_digit = 0U;
    after_decimal = 0U;

    if (*text == '-')
    {
        sign = -1.0f;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    while (*text != '\0')
    {
        if (*text == '.')
        {
            if (after_decimal != 0U)
            {
                return 0U;
            }
            after_decimal = 1U;
        }
        else if ((*text >= '0') && (*text <= '9'))
        {
            has_digit = 1U;
            if (after_decimal == 0U)
            {
                result = result * 10.0f + (float)(*text - '0');
            }
            else
            {
                result += (float)(*text - '0') * fraction_scale;
                fraction_scale *= 0.1f;
            }
        }
        else
        {
            return 0U;
        }
        text++;
    }

    if (has_digit == 0U)
    {
        return 0U;
    }
    *value = result * sign;
    return 1U;
}

static const char *Debug_StateName(BalanceState state)
{
    switch (state)
    {
        case BALANCE_STATE_DISABLED:
            return "OFF";
        case BALANCE_STATE_WAITING:
            return "WAIT";
        case BALANCE_STATE_ACTIVE:
            return "RUN";
        case BALANCE_STATE_PREDICTING:
            return "PRED";
        case BALANCE_STATE_FAILSAFE:
            return "FAIL";
        default:
            return "UNKNOWN";
    }
}

static void Debug_SendError(const char *message)
{
    DebugSerial_SendString("ERR ");
    DebugSerial_SendString(message);
    DebugSerial_SendString("\r\n");
}

static void Debug_ProcessCommand(char *line, uint32_t now_ms)
{
    char *tokens[DEBUG_MAX_TOKENS];
    uint8_t token_count;
    int32_t first_integer;
    int32_t second_integer;
    float kp;
    float ki;
    float kd;

    token_count = Debug_SplitTokens(line, tokens, DEBUG_MAX_TOKENS);
    if (token_count == 0U)
    {
        return;
    }

    if (Debug_StringEqualsIgnoreCase(tokens[0], "TARGET") != 0U)
    {
        if ((token_count != 3U)
            || (Debug_ParseInt32(tokens[1], &first_integer) == 0U)
            || (Debug_ParseInt32(tokens[2], &second_integer) == 0U)
            || (first_integer < 0) || (first_integer > 319)
            || (second_integer < 0) || (second_integer > 319))
        {
            Debug_SendError("TARGET range: 0..319");
            return;
        }

        BalanceControl_SetTarget((uint16_t)first_integer, (uint16_t)second_integer);
        BalanceProtocol_SetTarget((uint16_t)first_integer,
                                  (uint16_t)second_integer, now_ms);
        DebugSerial_SendString("OK TARGET\r\n");
    }
    else if (Debug_StringEqualsIgnoreCase(tokens[0], "PID") != 0U)
    {
        if ((token_count != 4U)
            || (Debug_ParseFloat(tokens[1], &kp) == 0U)
            || (Debug_ParseFloat(tokens[2], &ki) == 0U)
            || (Debug_ParseFloat(tokens[3], &kd) == 0U)
            || (kp < 0.0f) || (kp > 2.0f)
            || (ki < 0.0f) || (ki > 1.0f)
            || (kd < 0.0f) || (kd > 2.0f))
        {
            Debug_SendError("PID range: kp<=2 ki<=1 kd<=2");
            return;
        }

        BalanceControl_SetPID(kp, ki, kd);
        DebugSerial_SendString("OK PID\r\n");
    }
    else if (Debug_StringEqualsIgnoreCase(tokens[0], "ENABLE") != 0U)
    {
        if (token_count != 1U)
        {
            Debug_SendError("ENABLE takes no parameter");
            return;
        }
        BalanceControl_SetEnabled(1U);
        DebugSerial_SendString("OK ENABLE\r\n");
    }
    else if (Debug_StringEqualsIgnoreCase(tokens[0], "DISABLE") != 0U)
    {
        if (token_count != 1U)
        {
            Debug_SendError("DISABLE takes no parameter");
            return;
        }
        BalanceControl_SetEnabled(0U);
        DebugSerial_SendString("OK DISABLE\r\n");
    }
    else if (Debug_StringEqualsIgnoreCase(tokens[0], "NEUTRAL") != 0U)
    {
        if (token_count != 1U)
        {
            Debug_SendError("NEUTRAL takes no parameter");
            return;
        }
        BalanceControl_RequestNeutral();
        DebugSerial_SendString("OK NEUTRAL\r\n");
    }
    else if (Debug_StringEqualsIgnoreCase(tokens[0], "DIRECTION") != 0U)
    {
        if ((token_count != 2U)
            || (Debug_ParseInt32(tokens[1], &first_integer) == 0U)
            || ((first_integer != 1) && (first_integer != -1)))
        {
            Debug_SendError("DIRECTION must be 1 or -1");
            return;
        }
        BalanceControl_SetServoDirection((int8_t)first_integer);
        DebugSerial_SendString("OK DIRECTION\r\n");
    }
    else if (Debug_StringEqualsIgnoreCase(tokens[0], "STATUS") != 0U)
    {
        if (token_count != 1U)
        {
            Debug_SendError("STATUS takes no parameter");
            return;
        }
        DebugConsole_SendStatus();
    }
    else if (Debug_StringEqualsIgnoreCase(tokens[0], "HELP") != 0U)
    {
        if (token_count != 1U)
        {
            Debug_SendError("HELP takes no parameter");
            return;
        }
        DebugConsole_SendHelp();
    }
    else
    {
        Debug_SendError("unknown command, use HELP");
    }
}

void DebugConsole_Init(void)
{
    g_line_length = 0U;
    g_line_overflow_count = 0U;
    g_line_buffer[0] = '\0';
}

void DebugConsole_Process(uint32_t now_ms)
{
    uint8_t byte;

    while (DebugSerial_ReadByte(&byte) != 0U)
    {
        if ((byte == '\r') || (byte == '\n'))
        {
            if (g_line_length > 0U)
            {
                g_line_buffer[g_line_length] = '\0';
                Debug_ProcessCommand(g_line_buffer, now_ms);
                g_line_length = 0U;
            }
        }
        else if ((byte == 0x08U) || (byte == 0x7FU))
        {
            if (g_line_length > 0U)
            {
                g_line_length--;
            }
        }
        else if ((byte >= 0x20U) && (byte <= 0x7EU))
        {
            if (g_line_length < (DEBUG_LINE_BUFFER_SIZE - 1U))
            {
                g_line_buffer[g_line_length++] = (char)byte;
            }
            else
            {
                g_line_length = 0U;
                g_line_overflow_count++;
                Debug_SendError("command too long");
            }
        }
    }
}

void DebugConsole_SendStatus(void)
{
    BalanceControlStatus status;
    const BalanceProtocolStatistics *statistics;

    BalanceControl_GetStatus(&status);
    statistics = BalanceProtocol_GetStatistics();

    DebugSerial_SendString("STATE=");
    DebugSerial_SendString(Debug_StateName(status.state));
    DebugSerial_SendString(" ENABLE=");
    DebugSerial_SendUInt32(status.enabled);
    DebugSerial_SendString(" DIR=");
    DebugSerial_SendInt32(status.servo_direction);
    DebugSerial_SendString("\r\nTARGET=");
    DebugSerial_SendUInt32(status.target_x);
    DebugSerial_SendChar(',');
    DebugSerial_SendUInt32(status.target_y);
    DebugSerial_SendString(" BALL=");
    DebugSerial_SendUInt32(status.ball_x);
    DebugSerial_SendChar(',');
    DebugSerial_SendUInt32(status.ball_y);
    DebugSerial_SendString(" CONF=");
    DebugSerial_SendUInt32(status.confidence_per_mille);
    DebugSerial_SendString("\r\nERROR=");
    DebugSerial_SendFloat(status.error, 1U);
    DebugSerial_SendString(" P=");
    DebugSerial_SendFloat(status.p_term, 3U);
    DebugSerial_SendString(" I=");
    DebugSerial_SendFloat(status.i_term, 3U);
    DebugSerial_SendString(" D=");
    DebugSerial_SendFloat(status.d_term, 3U);
    DebugSerial_SendString(" OUT=");
    DebugSerial_SendFloat(status.pid_output, 3U);
    DebugSerial_SendString(" LIMIT=");
    DebugSerial_SendFloat(status.distance_tilt_limit, 2U);
    DebugSerial_SendString("\r\nANGLE=");
    DebugSerial_SendFloat(status.current_servo_angle, 1U);
    DebugSerial_SendString(" DESIRED=");
    DebugSerial_SendFloat(status.desired_servo_angle, 1U);
    DebugSerial_SendString(" PULSE=");
    DebugSerial_SendUInt32(status.current_servo_pulse_us);
    DebugSerial_SendString("\r\nFRAME=");
    DebugSerial_SendUInt32(statistics->valid_frames);
    DebugSerial_SendString(" CRC=");
    DebugSerial_SendUInt32(statistics->crc_errors);
    DebugSerial_SendString(" UART_OVF=");
    DebugSerial_SendUInt32(K230Serial_GetOverflowCount());
    DebugSerial_SendString(" DBG_OVF=");
    DebugSerial_SendUInt32(DebugSerial_GetOverflowCount());
    DebugSerial_SendString(" LINE_OVF=");
    DebugSerial_SendUInt32(g_line_overflow_count);
    DebugSerial_SendString(" FAILSAFE=");
    DebugSerial_SendUInt32(status.failsafe_count);
    DebugSerial_SendString(" TIMEOUT=");
    DebugSerial_SendUInt32(status.communication_timeout_count);
    DebugSerial_SendString(" UART_LOST=");
    DebugSerial_SendUInt32(status.communication_timed_out);
    DebugSerial_SendString(" ACK=");
    DebugSerial_SendUInt32(BalanceProtocol_IsTargetAcknowledged());
    DebugSerial_SendString("\r\n");
}

void DebugConsole_SendPeriodicStatus(void)
{
    BalanceControlStatus status;

    BalanceControl_GetStatus(&status);
    DebugSerial_SendString("S=");
    DebugSerial_SendString(Debug_StateName(status.state));
    DebugSerial_SendString(" X=");
    DebugSerial_SendUInt32(status.ball_x);
    DebugSerial_SendString(" T=");
    DebugSerial_SendUInt32(status.target_x);
    DebugSerial_SendString(" E=");
    DebugSerial_SendFloat(status.error, 1U);
    DebugSerial_SendString(" A=");
    DebugSerial_SendFloat(status.current_servo_angle, 1U);
    DebugSerial_SendString("\r\n");
}

void DebugConsole_SendHelp(void)
{
    DebugSerial_SendString(
        "TARGET x y        set target 0..319\r\n"
        "PID kp ki kd      set gains\r\n"
        "ENABLE / DISABLE  control switch\r\n"
        "NEUTRAL           reset PID and return center\r\n"
        "DIRECTION 1|-1    servo direction\r\n"
        "STATUS            full status\r\n"
        "HELP              command list\r\n");
}

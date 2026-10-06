#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "StatusLED.h"
#include "K230Serial.h"
#include "DebugSerial.h"
#include "Servo.h"
#include "SystemTimer.h"
#include "BalanceProtocol.h"
#include "BalanceControl.h"
#include "DebugConsole.h"

/* 主循环任务周期。所有周期判断均使用无符号时间差，因此兼容毫秒计数回绕。 */
#define CONTROL_TASK_PERIOD_MS       20U
#define OLED_TASK_PERIOD_MS          200U
#define DEBUG_TASK_PERIOD_MS         500U

/**
  * @brief 返回OLED使用的4字符状态缩写。
  * @note  OLED字体库只包含ASCII字符，因此显示使用英文缩写，代码注释保持中文。
  */
static const char *Main_GetStateText(BalanceState state,
                                     uint8_t communication_timed_out)
{
    switch (state)
    {
        case BALANCE_STATE_DISABLED:
            return "OFF ";
        case BALANCE_STATE_WAITING:
            return "WAIT";
        case BALANCE_STATE_ACTIVE:
            return "RUN ";
        case BALANCE_STATE_PREDICTING:
            return "PRED";
        case BALANCE_STATE_FAILSAFE:
            return (communication_timed_out != 0U) ? "UART" : "LOST";
        default:
            return "ERR ";
    }
}

/**
  * @brief 按5 Hz刷新OLED上的核心控制状态。
  * @note  现有OLED使用软件I2C，刷新期间会占用CPU，因此不能逐控制周期刷新。
  */
static void Main_UpdateOLED(void)
{
    BalanceControlStatus status;
    const BalanceProtocolStatistics *statistics;
    int32_t error_integer;
    uint32_t angle_integer;
    uint32_t kp_scaled;
    uint32_t kd_scaled;
    float kp;
    float ki;
    float kd;

    BalanceControl_GetStatus(&status);
    BalanceControl_GetPID(&kp, &ki, &kd);
    statistics = BalanceProtocol_GetStatistics();

    error_integer = (int32_t)status.error;
    angle_integer = (uint32_t)(status.current_servo_angle + 0.5f);
    kp_scaled = (uint32_t)(kp * 1000.0f + 0.5f);
    kd_scaled = (uint32_t)(kd * 1000.0f + 0.5f);

    /* 每行先写16个空格，避免新内容比旧内容短时留下残字符。 */
    OLED_ShowString(1, 1, "                ");
    OLED_ShowString(2, 1, "                ");
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(4, 1, "                ");

    OLED_ShowString(1, 1, "X:");
    if (status.ball_x <= 319U)
    {
        OLED_ShowNum(1, 3, status.ball_x, 3);
    }
    else
    {
        OLED_ShowString(1, 3, "---");
    }
    OLED_ShowString(1, 7, "T:");
    OLED_ShowNum(1, 9, status.target_x, 3);

    OLED_ShowString(2, 1, "E:");
    OLED_ShowSignedNum(2, 3, error_integer, 3);
    OLED_ShowString(2, 8, "A:");
    OLED_ShowNum(2, 10, angle_integer, 3);

    /* 为节省屏幕宽度，KP和KD显示为实际值乘1000，例如0.060显示为060。 */
    OLED_ShowString(3, 1, "KP:");
    OLED_ShowNum(3, 4, kp_scaled, 3);
    OLED_ShowString(3, 8, "KD:");
    OLED_ShowNum(3, 11, kd_scaled, 3);

    OLED_ShowString(4, 1, Main_GetStateText(
        status.state, status.communication_timed_out));
    OLED_ShowString(4, 6, "C:");
    OLED_ShowNum(4, 8, statistics->crc_errors % 1000U, 3);
    OLED_ShowString(4, 12,
                    BalanceProtocol_IsTargetAcknowledged() ? "ACK" : "---");
}

/**
  * @brief 根据平衡状态生成不同的PA1 LED指示模式。
  * @note  该函数不使用Delay，不会阻塞串口解析和PID任务。
  */
static void Main_UpdateStatusLED(uint32_t now_ms)
{
    BalanceControlStatus status;
    uint8_t led_on;

    BalanceControl_GetStatus(&status);
    led_on = 0U;

    switch (status.state)
    {
        case BALANCE_STATE_DISABLED:
            led_on = 0U;
            break;

        case BALANCE_STATE_WAITING:
            /* 每秒亮100 ms，表示系统运行但正在等待足够的真实视觉帧。 */
            led_on = (uint8_t)((now_ms % 1000U) < 100U);
            break;

        case BALANCE_STATE_ACTIVE:
            led_on = 1U;
            break;

        case BALANCE_STATE_PREDICTING:
            /* 预测状态以200 ms为一个亮灭周期。 */
            led_on = (uint8_t)(((now_ms / 100U) & 1U) != 0U);
            break;

        case BALANCE_STATE_FAILSAFE:
        default:
            /* 失效保护快速闪烁，便于现场区分正常等待与通信/丢球故障。 */
            led_on = (uint8_t)(((now_ms / 50U) & 1U) != 0U);
            break;
    }

    StatusLED_Set(led_on);
}

int main(void)
{
    uint32_t now_ms;
    uint32_t last_control_ms;
    uint32_t last_oled_ms;
    uint32_t last_debug_ms;
    BalanceBallState ball_state;

    /* 统一设置中断优先级分组：2位抢占优先级、2位响应优先级。 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    StatusLED_Init();
    StatusLED_Set(1U);
    OLED_Init();
    OLED_ShowString(1, 1, "Ball Balance");
    OLED_ShowString(2, 1, "Initializing");

    /*
     * 串口必须先于协议层初始化；PWM必须先于控制器开始运行。
     * 1 ms系统时基最后启动，避免把OLED上电延时计入控制超时。
     */
    K230Serial_Init();
    DebugSerial_Init();
    Servo_Init();
    BalanceProtocol_Init();
    BalanceControl_Init();
    DebugConsole_Init();
    SystemTimer_Init();

    Delay_ms(100U);
    StatusLED_Set(0U);
    OLED_Clear();

    now_ms = SystemTimer_GetMilliseconds();
    last_control_ms = now_ms;
    last_oled_ms = now_ms;
    last_debug_ms = now_ms;

    /* 上电立即把默认目标点发送给K230；未收到ACK时协议层每500 ms重发。 */
    BalanceProtocol_SetTarget(BALANCE_DEFAULT_TARGET_X,
                              BALANCE_DEFAULT_TARGET_Y, now_ms);

    DebugSerial_SendString("\r\nSTM32F103 steel-ball balance controller started\r\n");
    DebugConsole_SendHelp();

    while (1)
    {
        now_ms = SystemTimer_GetMilliseconds();

        /* 高频、非阻塞任务：尽快清空两个串口的接收环形缓冲区。 */
        BalanceProtocol_Process(now_ms);
        while (BalanceProtocol_GetNewBallState(&ball_state) != 0U)
        {
            BalanceControl_OnBallState(&ball_state, now_ms);
        }
        DebugConsole_Process(now_ms);
        BalanceProtocol_Service(now_ms);

        if (SystemTimer_Elapsed(now_ms, last_control_ms) >= CONTROL_TASK_PERIOD_MS)
        {
            /*
             * 直接把基准更新到当前时间，避免主循环偶尔变慢后连续补跑多次PID，
             * 因为补跑时并没有新的视觉测量，连续计算反而会放大瞬态误差。
             */
            last_control_ms = now_ms;
            BalanceControl_Run20ms(now_ms);
        }

        if (SystemTimer_Elapsed(now_ms, last_oled_ms) >= OLED_TASK_PERIOD_MS)
        {
            last_oled_ms = now_ms;
            Main_UpdateOLED();
        }

        if (SystemTimer_Elapsed(now_ms, last_debug_ms) >= DEBUG_TASK_PERIOD_MS)
        {
            last_debug_ms = now_ms;
            DebugConsole_SendPeriodicStatus();
        }

        Main_UpdateStatusLED(now_ms);
    }
}

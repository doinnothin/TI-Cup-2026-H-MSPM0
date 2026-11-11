#include "control.h"
#include "pid.h"
#include "motor.h"
#include "gyro.h"
#include "huidu.h"
#include "delay.h"

volatile uint8_t control_enable = 0U;
volatile uint8_t control_auto_enable = 0U;
volatile uint8_t control_yaw_enable = 0U;
volatile uint8_t control_trace_enable = 0U;
volatile float control_target_yaw = 0.0f;
volatile float control_trace_error = 0.0f;

static float base_speed = 120.0f;
static float trace_start_yaw = 0.0f;
static float trace_filter_error = 0.0f;
static uint8_t white_count = 0U;
static uint8_t black_count = 0U;
static uint8_t straight_guard = 0U;
static volatile uint8_t control_event = CONTROL_EVENT_NONE;

static void Control_SetTarget(float left, float right)
{
    if (left < 0.0f)
    {
        left = 0.0f;
    }
    if (right < 0.0f)
    {
        right = 0.0f;
    }
    if (left > MOTOR_TARGET_MAX)
    {
        left = MOTOR_TARGET_MAX;
    }
    if (right > MOTOR_TARGET_MAX)
    {
        right = MOTOR_TARGET_MAX;
    }
    motorA.target = left;
    motorB.target = right;
}

static void Control_ResetUpper(void)
{
    pid_init(&angle, POSITION_PID, YAW_P, 0.0f, YAW_D);
    pid_init(&trace_pid, POSITION_PID, TRACE_P, 0.0f, TRACE_D);
    trace_filter_error = 0.0f;
    control_trace_error = 0.0f;
}

void Control_Start(float yaw, float speed)
{
    /* 速度环使用增量式 PID。 */
    pid_init(&motorA, DELTA_PID, SPEED_P, SPEED_I, SPEED_D);
    pid_init(&motorB, DELTA_PID, SPEED_P, SPEED_I, SPEED_D);
    /* 航向环和灰度环使用位置式 PID。 */
    Control_ResetUpper();

    control_target_yaw = yaw;
    base_speed = speed;
    control_enable = 1U;
    control_auto_enable = 0U;
    control_yaw_enable = 0U;
    control_trace_enable = 0U;
    control_event = CONTROL_EVENT_NONE;
    white_count = 0U;
    black_count = 0U;
    straight_guard = 0U;
    Control_SetTarget(speed, speed);
}

void Control_QuickBrake(void)
{
    control_enable = 0U;
    control_auto_enable = 0U;
    control_yaw_enable = 0U;
    control_trace_enable = 0U;
    motorA.out = 0.0f;
    motorB.out = 0.0f;
    Motor_Brake();
}

void Control_Stop(void)
{
    Control_QuickBrake();
    delay_ms(STOP_BRAKE_MS);
    Motor_Stop();
}

void Control_YawStart(float yaw, float speed)
{
    Control_Start(yaw, speed);
    control_yaw_enable = 1U;
}

void Control_AutoStart(float speed)
{
    Control_YawStart(Yaw(), speed);
    control_auto_enable = 1U;
}

void Control_TraceStart(float speed)
{
    Control_Start(Yaw(), speed);
    Control_ToTrace();
}

void Control_SetBaseSpeed(float speed)
{
    if (speed < 0.0f)
    {
        speed = 0.0f;
    }
    else if (speed > MOTOR_TARGET_MAX)
    {
        speed = MOTOR_TARGET_MAX;
    }
    base_speed = speed;
}

uint8_t Control_StraightRun(void)
{
    uint8_t sensor;

    angle.target = control_target_yaw;
    angle.now = Yaw();
    pid_cal_angle(&angle);

    if ((angle.error[0] < YAW_DEADBAND) && (angle.error[0] > -YAW_DEADBAND))
    {
        Control_SetTarget(base_speed, base_speed);
    }
    else
    {
        Control_SetTarget(base_speed + angle.out * YAW_K, base_speed - angle.out * YAW_K);
    }

    if (straight_guard != 0U)
    {
        --straight_guard;
        black_count = 0U;
        return 0U;
    }

    sensor = Huidu_Read();
    if (sensor != SENSOR_ALL_WHITE)
    {
        if (++black_count >= BLACK_CONFIRM_COUNT)
        {
            black_count = 0U;
            return 1U;
        }
    }
    else
    {
        black_count = 0U;
    }
    return 0U;
}

uint8_t Control_TraceRun(void)
{
    uint8_t sensor = Huidu_Read();
    float raw_error;

    if (sensor == SENSOR_ALL_WHITE)
    {
        if (++white_count >= WHITE_CONFIRM_COUNT)
        {
            white_count = 0U;
            return 1U;
        }
    }
    else
    {
        white_count = 0U;
    }

    /* S4、S5 或 S4+S5 表示黑线已处于几何居中。
     * 不要让旧的滤波误差或 D 项在直道上持续修正左右：
     * 而是立即下发左右相等的轮速。 */
    if (Huidu_IsCentered(sensor) != 0U)
    {
        trace_filter_error = 0.0f;
        control_trace_error = 0.0f;
        pid_init(&trace_pid, POSITION_PID, TRACE_P, 0.0f, TRACE_D);
        Control_SetTarget(base_speed, base_speed);
        return 0U;
    }

    raw_error = Huidu_GetError(sensor);
    trace_filter_error += TRACE_FILTER_ALPHA * (raw_error - trace_filter_error);
    trace_pid.now = trace_filter_error;
    control_trace_error = trace_pid.now;
    pid_cal_trace(&trace_pid);
    /* 与 24H_N 工程 T4 黑线循迹保持相同的左右轮差速方向。 */
    Control_SetTarget(base_speed + trace_pid.out, base_speed - trace_pid.out);
    return 0U;
}

void Control_ToTrace(void)
{
    control_yaw_enable = 0U;
    control_trace_enable = 1U;
    trace_start_yaw = Yaw();
    pid_init(&trace_pid, POSITION_PID, TRACE_P, 0.0f, TRACE_D);
    trace_filter_error = 0.0f;
    control_trace_error = 0.0f;
    white_count = 0U;
    black_count = 0U;
    Control_SetTarget(base_speed, base_speed);
}

void Control_ToStraight(void)
{
    control_trace_enable = 0U;
    control_yaw_enable = 1U;
    control_target_yaw = trace_start_yaw + 179.0f;
    if (control_target_yaw > 180.0f)
    {
        control_target_yaw -= 360.0f;
    }
    pid_init(&angle, POSITION_PID, YAW_P, 0.0f, YAW_D);
    control_trace_error = 0.0f;
    white_count = 0U;
    black_count = 0U;
    straight_guard = STRAIGHT_GUARD_COUNT;
    Control_SetTarget(base_speed, base_speed);
}

void Control_UpperRun(void)
{
    if (control_enable == 0U)
    {
        return;
    }

    if (control_yaw_enable != 0U)
    {
        if ((control_auto_enable != 0U) && (Control_StraightRun() != 0U))
        {
            control_event = CONTROL_EVENT_BLACK;
        }
        else if (control_auto_enable == 0U)
        {
            Control_StraightRun();
        }
    }
    else if (control_trace_enable != 0U)
    {
        if ((control_auto_enable != 0U) && (Control_TraceRun() != 0U))
        {
            control_event = CONTROL_EVENT_WHITE;
        }
        else if (control_auto_enable == 0U)
        {
            Control_TraceRun();
        }
    }
}

uint8_t Control_GetEvent(void)
{
    uint8_t event = control_event;
    control_event = CONTROL_EVENT_NONE;
    return event;
}

void Control_TimerInit(void)
{
    DL_TimerG_clearInterruptStatus(Timer_Control_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(Timer_Control_INST_INT_IRQN);
    NVIC_EnableIRQ(Timer_Control_INST_INT_IRQN);
    DL_TimerG_startCounter(Timer_Control_INST);
}

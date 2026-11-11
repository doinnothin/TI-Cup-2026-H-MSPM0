#include "ti_msp_dl_config.h"

#include <math.h>
#include <stdio.h>

#include "bsp/control.h"
#include "bsp/app_clock.h"
#include "bsp/interrupt.h"
#include "bsp/lap_trace.h"
#include "code/delay.h"
#include "code/debug_log.h"
#include "code/encoder.h"
#include "code/licheng.h"
#include "code/motor.h"
#include "code/oled.h"
#include "code/vision_link.h"
#include "app/ball_actuator/ball_actuator.h"
#include "app/ball_actuator/pendulum_calibration.h"
#include "app/app_key_input.h"
#include "app/app_status_display.h"
#include "app/app_telemetry.h"
#include "app/q3/ball_position_control.h"
#include "app/q3/ball_control_config.h"
#include "app/q3/ball_mechanism_id.h"
#include "app/q3/pendulum_geometry_calibration.h"
#include "app/q3/ball_pulse_id.h"
#include "app/q3/q3_sequence.h"
#include "app/q4/q45_vehicle_control.h"
#include "app/q6/q6_config.h"

/*
 * 10 ms 协作式调度器：底盘功能与球位置控制
 * 均为应用任务。电机 UART RX 仍由中断驱动。
 */

static uint8_t g_stop_handled;

typedef enum
{
    APP_TASK_NONE = 0,
    APP_TASK_Q2,
    APP_TASK_Q3,
    APP_TASK_Q4,
    APP_TASK_Q5,
    APP_TASK_Q6
} AppTaskMode;

/* Q3 保持其控制器与参数冻结不变。此小型 UI 标志仅
 * 实现 Q3/Q4/Q5 通用的两次按键启动契约。 */
static AppTaskMode g_selected_task;
static bool g_q3_launch_armed;
#if Q6_TASK_ENABLE && Q6_BOOT_AUTO_SELECT
static bool g_q6_boot_selected;
#endif
#if Q6_TASK_ENABLE
static bool g_q6_pb24_start_pending;
#endif
/* 首次按下 PA31 会锁存一个请求。该请求一直保持，直到 X42S
 * 上报就绪并确认新的当前位置零点标记，因此复位后
 * 尽早按一次键无需再按第二次设置键。 */
static bool g_q3_zero_requested;
static uint32_t g_q3_started_ms;
static uint32_t g_q3_elapsed_ms;
static bool g_q3_timer_running;

static bool Q3Key_ConsumePress(uint32_t now_ms)
{
    static bool candidate;
    static bool stable;
    static uint32_t changed_ms;
    bool pressed = DL_GPIO_readPins(KEY_KEY_T4_PORT, KEY_KEY_T4_PIN) == 0U;
    g_key_t4 = 0U;
    if (pressed != candidate)
    {
        candidate = pressed;
        changed_ms = now_ms;
    }
    if (stable != candidate && now_ms - changed_ms >= 20U)
    {
        stable = candidate;
        return stable;
    }
    return false;
}

static const char *Lap_StateText(void)
{
    switch (LapTrace_GetState())
    {
    case LAP_TRACE_RUNNING:
        return "Q2 RUN";
    case LAP_TRACE_FINISHED:
        return "Q2 PASS";
    case LAP_TRACE_TIMEOUT:
        return "Q2 TIMEOUT";
    default:
        return "READY";
    }
}

static bool AppVisionAtCenter(uint32_t now_ms)
{
    VisionLinkSample sample;

    return VisionLink_GetLatest(&sample) && sample.valid && isfinite(sample.position_cm) &&
           (sample.confidence_pct >= BALL_CONTROL_CONFIDENCE_MIN) &&
           ((now_ms - sample.rx_time_ms) <= BALL_CONTROL_VISION_TIMEOUT_MS) &&
           (fabsf(sample.position_cm) <= BALL_CONTROL_START_WINDOW_CM);
}

static void Show_Status(void)
{
    Q45VehicleStatus vehicle;
    Q3SequenceStatus q3;
    char line[APP_STATUS_DISPLAY_LINE_CAPACITY];

    Q45Vehicle_GetStatus(&vehicle);
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MSPM0 26HT", 12);
    if (((g_selected_task == APP_TASK_Q4) || (g_selected_task == APP_TASK_Q5) ||
         (g_selected_task == APP_TASK_Q6)) &&
        (vehicle.mode != Q45_VEHICLE_MODE_NONE))
    {
        AppStatusDisplay_ShowVehicle(&vehicle, g_selected_task == APP_TASK_Q6, line);
    }
    else if (g_selected_task == APP_TASK_Q3)
    {
        Q3Sequence_GetStatus(&q3);
        OLED_ShowString(0, 16, (u8 *)"Q3", 12);
        OLED_ShowString(32,
                        16,
                        (u8 *)(g_q3_zero_requested ? "ZEROING"
                               : g_q3_launch_armed
                                   ? (AppVisionAtCenter(AppClock_NowMs()) ? "READY" : "CENTER")
                                   : Q3Sequence_StateName(q3.state)),
                        12);
        (void)snprintf(line,
                       sizeof(line),
                       "T:%lus %s",
                       (unsigned long)(g_q3_elapsed_ms / 1000U),
                       g_q3_zero_requested ? "WAIT" : (g_q3_launch_armed ? "2ND KEY" : "1ST KEY"));
        OLED_ShowString(0, 32, (u8 *)line, 12);
    }
    else
    {
        OLED_ShowString(0, 16, (u8 *)"Q2 TRACE", 12);
        OLED_ShowString(0, 32, (u8 *)Lap_StateText(), 12);
    }
    OLED_Refresh();
}

static void Stop_All(void)
{
    if (control_enable != 0U)
    {
        Control_Stop();
    }
    LapTrace_Reset();
    if (Q45Vehicle_IsActive())
    {
        Q45Vehicle_Stop(AppClock_NowMs());
    }
    g_stop_handled = 1U;
}

static void Start_Q2(void)
{
    Stop_All();
    Q3Sequence_Stop(AppClock_NowMs());
    BallPositionControl_Stop(AppClock_NowMs());
    g_q3_launch_armed = false;
    g_q3_zero_requested = false;
    g_q3_started_ms = 0U;
    g_q3_elapsed_ms = 0U;
    g_q3_timer_running = false;
    g_selected_task = APP_TASK_Q2;
    g_stop_handled = 0U;
    Licheng_Reset();
    LapTrace_StartToDistance(LAP_TRACE_STOP_DISTANCE_CM, LAP_TRACE_TIME_LIMIT_MS);
    Control_TraceStart(LAP_TRACE_TARGET_SPEED);
}

static void Start_Q45(Q45VehicleMode mode, uint32_t now_ms)
{
    Q45VehicleStatus vehicle;

    Q45Vehicle_GetStatus(&vehicle);
    if ((vehicle.mode == mode) && Q45Vehicle_IsActive())
    {
        /* 同一选中按键的第二次动作只有一种：启动。在 READY 之前
         * 或运行期间按下会被有意忽略。 */
        if (vehicle.state == Q45_VEHICLE_READY)
        {
            (void)Q45Vehicle_StartSelected(now_ms);
        }
        return;
    }

    /* Q3 与车辆控制器绝不能同时发送 X42S 目标。 */
    Stop_All();
    Q3Sequence_Stop(now_ms);
    BallPositionControl_Stop(now_ms);
    BallMechanismId_Abort(now_ms);
    PendulumGeometryCalibration_Abort(now_ms);
    BallPulseId_Abort(now_ms);
    g_q3_launch_armed = false;
    g_q3_zero_requested = false;
    g_q3_started_ms = 0U;
    g_q3_elapsed_ms = 0U;
    g_q3_timer_running = false;
    g_selected_task = (mode == Q45_VEHICLE_MODE_Q4)
                          ? APP_TASK_Q4
                          : ((mode == Q45_VEHICLE_MODE_Q6) ? APP_TASK_Q6 : APP_TASK_Q5);
    if (Q45Vehicle_SelectMode(mode, now_ms))
    {
        g_stop_handled = 0U;
    }
}

int main(void)
{
    uint32_t display_last_ms = 0U;
    uint32_t app_time_ms = 0U;
    uint32_t key_start_last_ms = 0U;
    uint32_t key_t1_last_ms = 0U;
    uint32_t key_t2_last_ms = 0U;
    uint32_t key_t3_last_ms = 0U;
    uint32_t key_t4_last_ms = 0U;

    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    Licheng_Init();
    Control_TimerInit();
    OLED_Init();
    VisionLink_Init();
    DebugLog_Init();
    BallActuator_Init(1U);
    PendulumCalibration_Init();
    BallPositionControl_Init();
    BallMechanismId_Init();
    PendulumGeometryCalibration_Init();
    BallPulseId_Init();
    Q3Sequence_Init();
    Q45Vehicle_Init();
    LapTrace_Reset();

    enable_group1_irq = 1U;
    Interrupt_Init();

    while (1)
    {
        app_time_ms = AppClock_NowMs();
        AppTelemetry_BeginCycle();
        VisionLink_SetNow(app_time_ms);
        VisionLink_Poll();
        /* 快速路径：UART3 RX 已拼装出最新的视觉帧。
         * 先刷新 X42S 反馈，再计算/发送外环位置环，
         * 之后才执行任何可选的辨识或校准任务。 */
        BallActuator_Task(app_time_ms);
        BallPositionControl_Task(app_time_ms);
        Q3Sequence_Task(app_time_ms);
        if (g_q3_timer_running)
        {
            Q3SequenceStatus q3;

            Q3Sequence_GetStatus(&q3);
            if (q3.completion_ms != 0U)
            {
                /* 比赛计时在首次静止进入实测的 -5 +/- 1 cm
                 * 区间时结束。此后 X42S 可返回其 PA31
                 * 零点，而不会延长 OLED 秒表。 */
                g_q3_elapsed_ms = q3.completion_ms;
                g_q3_timer_running = false;
            }
            else
            {
                g_q3_elapsed_ms = app_time_ms - g_q3_started_ms;
                if (!Q3Sequence_IsActive())
                {
                    /* 在达到合格最终位置前出现故障/STOPPED 时也会
                     * 冻结秒表，而不是让其继续计时。 */
                    g_q3_timer_running = false;
                }
            }
        }
        /* Q4/Q5/Q6 车辆模式仅在其任务按键请求后才占用 X42S。
         * 已归档的静态恢复测试已从本镜像中编译剔除。 */
#if Q6_TASK_ENABLE && Q6_BOOT_AUTO_SELECT
        /* 仅台架镜像：无需按键即进入 Q6，使 flash 复位后
         * 能自主启动整个保持试验。 */
        if ((app_time_ms >= 1000U) && !g_q6_boot_selected)
        {
            g_q6_boot_selected = true;
#if (Q6_BENCH_EXPLICIT_TARGET_MM != -32768)
            /* 仅台架镜像：将保持目标固定为请求值；
             * 启动后由运动曲线将球带到该位置。 */
            (void)Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT,
                                               (float)Q6_BENCH_EXPLICIT_TARGET_MM / 10.0f);
#endif
            Start_Q45(Q45_VEHICLE_MODE_Q6, app_time_ms);
        }
#endif
        Q45Vehicle_Task(app_time_ms);
        DebugLog_PublishControlFrame(app_time_ms);

        /* 这些独立测试模块在 Q3 期间通常处于非活动状态。
         * 将它们放在快速路径之后，可避免新相机样本到达时
         * 增加控制延迟。 */
        PendulumCalibration_Task(app_time_ms);
        BallMechanismId_Task(app_time_ms);
        PendulumGeometryCalibration_Task(app_time_ms);
        BallPulseId_Task(app_time_ms);
        DebugLog_Task();

        /* 将慢速诊断发布保持在每轮循环队列排空之后。 */
        AppTelemetry_PublishDue(app_time_ms);

        if (AppKeyInput_ConsumePress(KEY_KEY_START_PORT,
                                     KEY_KEY_START_PIN,
                                     &g_key_start,
                                     app_time_ms,
                                     &key_start_last_ms))
        {
            Stop_All();
            Q3Sequence_Stop(app_time_ms);
            BallPositionControl_Stop(app_time_ms);
            BallMechanismId_Abort(app_time_ms);
            PendulumGeometryCalibration_Abort(app_time_ms);
            BallPulseId_Abort(app_time_ms);
            g_q3_launch_armed = false;
            g_q3_zero_requested = false;
            g_q3_started_ms = 0U;
            g_q3_elapsed_ms = 0U;
            g_q3_timer_running = false;
        }

        if (AppKeyInput_ConsumePress(
                KEY_KEY_T1_PORT, KEY_KEY_T1_PIN, &g_key_t1, app_time_ms, &key_t1_last_ms))
        {
            /* 在所有构建中 PB11 始终是 Q5 启动键。 */
            Start_Q45(Q45_VEHICLE_MODE_Q5, app_time_ms);
        }

        if (AppKeyInput_ConsumePress(
                KEY_KEY_T2_PORT, KEY_KEY_T2_PIN, &g_key_t2, app_time_ms, &key_t2_last_ms))
        {
#if Q6_TASK_ENABLE
            DebugLog_PublishQ6Event(app_time_ms, 1U, 0);
            /* PB24 请求启动 Q6；在 CAPTURE 采集稳定目标窗口期间
             * 保持该请求，随后只启动一次。 */
            Start_Q45(Q45_VEHICLE_MODE_Q6, app_time_ms);
            g_q6_pb24_start_pending = true;
#else
            /* 在其他镜像中 PB24 保留其独立的 Q2 底盘练习功能。 */
            Start_Q2();
#endif
        }

        /* PA28：比赛 Q4，在球保持 O 点的同时从 A 到 B。 */
        if (AppKeyInput_ConsumePress(
                KEY_KEY_T3_PORT, KEY_KEY_T3_PIN, &g_key_t3, app_time_ms, &key_t3_last_ms))
        {
            Start_Q45(Q45_VEHICLE_MODE_Q4, app_time_ms);
        }

        /* 首次按下 PA31 会请求一次新的物理零点。该请求
         * 在电机启动/使能反馈期间保持有效，避免出现用户
         * 在复位后首次 X42S 状态轮询之前按下 PA31 的竞态。 */
        if (g_q3_zero_requested && !Q3Sequence_IsActive())
        {
            if (Q3Sequence_ZeroReference(app_time_ms))
            {
                g_q3_zero_requested = false;
                g_q3_launch_armed = true;
                g_q3_started_ms = 0U;
                g_q3_elapsed_ms = 0U;
                g_q3_timer_running = false;
                g_stop_handled = 0U;
            }
        }

        if (Q3Key_ConsumePress(app_time_ms) && !Q3Sequence_IsActive())
        {
            if (Q45Vehicle_IsActive())
            {
                Stop_All();
            }
            g_selected_task = APP_TASK_Q3;
            if (!g_q3_launch_armed)
            {
                /* 首次按下 PA31 总是基于当前 X42S 位姿创建新的
                 * 会话原点，即使之前已执行过 Q3 也是如此。 */
                Q3Sequence_Stop(app_time_ms);
                BallPositionControl_Stop(app_time_ms);
                BallMechanismId_Abort(app_time_ms);
                PendulumGeometryCalibration_Abort(app_time_ms);
                BallPulseId_Abort(app_time_ms);
                g_q3_zero_requested = true;
            }
            else if (AppVisionAtCenter(app_time_ms) && Q3Sequence_Start(app_time_ms))
            {
                /* 正式 Q3 执行仅在第二次按下 PA31 时开始，
                 * 且仅在视觉确认 |x| <= 1 cm 之后。 */
                Q3Sequence_EnableAutoCycle(false);
                g_q3_launch_armed = false;
                g_q3_started_ms = app_time_ms;
                g_q3_elapsed_ms = 0U;
                g_q3_timer_running = true;
                g_stop_handled = 0U;
            }
        }

        if ((LapTrace_StopRequested() != 0U) && (g_stop_handled == 0U))
        {
            if (control_enable != 0U)
            {
                Control_Stop();
            }
            LapTrace_Reset();
            g_stop_handled = 1U;
        }

        if (Q45Vehicle_StopRequested() && (g_stop_handled == 0U))
        {
            if (control_enable != 0U)
            {
                Control_Stop();
            }
            g_stop_handled = 1U;
        }

#if Q6_TASK_ENABLE
        if (g_q6_pb24_start_pending)
        {
            Q45VehicleStatus vehicle;

            Q45Vehicle_GetStatus(&vehicle);
            if ((vehicle.mode != Q45_VEHICLE_MODE_Q6) || (vehicle.state >= Q45_VEHICLE_RUNNING))
            {
                g_q6_pb24_start_pending = false;
            }
            else if ((vehicle.state == Q45_VEHICLE_READY) && vehicle.center_ready)
            {
                Start_Q45(Q45_VEHICLE_MODE_Q6, app_time_ms);
                Q45Vehicle_GetStatus(&vehicle);
                g_q6_pb24_start_pending = (vehicle.state == Q45_VEHICLE_READY);
            }
        }
#endif

        /* OLED 刷新保持低频，不影响 10 ms 控制定时。 */
#if Q6_TASK_ENABLE && Q6_BENCH_AUTO_START
        if (g_selected_task == APP_TASK_Q6)
        {
            Q45VehicleStatus vehicle;

            Q45Vehicle_GetStatus(&vehicle);
            if ((vehicle.mode == Q45_VEHICLE_MODE_Q6) && (vehicle.state == Q45_VEHICLE_READY))
            {
                /* 仅台架镜像：采集到目标后自动启动运行；
                 * StartSelected() 会在内部重新检查就绪状态。 */
                (void)Q45Vehicle_StartSelected(app_time_ms);
            }
        }
#endif

        if ((app_time_ms - display_last_ms) >= 1000U)
        {
            display_last_ms = app_time_ms;
            Show_Status();
        }
    }
}

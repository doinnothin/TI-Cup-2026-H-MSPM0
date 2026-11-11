#include "q3_sequence.h"
#include <math.h>
#include <string.h>
#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/ball_position_control.h"
#include "app/q3/ball_control_config.h"
#include "app/q3/q3_control_config.h"

static Q3SequenceStatus s_status;
static bool s_result_log_pending;
static uint32_t s_zero_ms;
static float s_return_reference_cm;
static uint32_t s_return_reference_update_ms;
static uint8_t s_plus5_confirm_frames;
static uint32_t s_return_zero_retry_ms;
static uint32_t s_zero_prepare_retry_ms;

static void Enter(Q3SequenceState state, uint32_t now);

static float Q3Sequence_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool Q3Sequence_FinalPositionAccepted(const BallPositionControlStatus *control)
{
    /* target_reached 已要求较小的测量速度和一个
     * 连续的静止窗口。此处将球本身与 -5 cm 比较；
     * 当摩擦已使球静止在 -5.5 cm 时，移动的返回参考
     * 可能合法地为 -4.9 cm，而该位置处于
     * 比赛的 -5 +/- 1 cm 接受带内。 */
    return (control != NULL) && control->target_reached &&
           (Q3Sequence_Abs(control->measured_cm - Q3_RETURN_FINAL_TARGET_CM) <=
            Q3_FINAL_ACCEPTANCE_TOLERANCE_CM);
}

static void Q3Sequence_RecordFinalCompletion(uint32_t now)
{
    s_status.completion_ms = now - s_status.motion_started_ms;
    s_status.time_pass = s_status.completion_ms <= 5000U;
    BallPositionControl_SetOuterProfile(BALL_OUTER_PROFILE_RETURN_HOLD);
    Enter(Q3_SEQUENCE_FINAL_HOLD, now);
}

static bool Q3Sequence_AdvanceReturnReference(uint32_t now)
{
    float speed_cm_s;
    float step_cm;
    float next_reference;

    if ((now - s_return_reference_update_ms) < Q3_RETURN_REFERENCE_UPDATE_MS)
    {
        return true;
    }
    speed_cm_s = (s_status.state == Q3_SEQUENCE_RETURN_BRAKE)
                     ? Q3_RETURN_BRAKE_REFERENCE_SPEED_CM_S
                     : Q3_RETURN_CRUISE_REFERENCE_SPEED_CM_S;
    step_cm = speed_cm_s * (float)(now - s_return_reference_update_ms) / 1000.0f;
    if (step_cm < 0.0f)
        step_cm = 0.0f;
    if (step_cm > 1.0f)
        step_cm = 1.0f;
    next_reference = s_return_reference_cm - step_cm;
    if (next_reference < Q3_RETURN_FINAL_TARGET_CM)
    {
        next_reference = Q3_RETURN_FINAL_TARGET_CM;
    }
    s_return_reference_update_ms = now;
    if (Q3Sequence_Abs(next_reference - s_return_reference_cm) <= 0.001f)
    {
        return true;
    }
    s_return_reference_cm = next_reference;
    return BallPositionControl_SetTargetCm(s_return_reference_cm, now);
}

static bool Q3Sequence_ShouldBrake(const BallPositionControlStatus *control)
{
    float remaining_cm;
    float speed_cm_s;
    float stopping_distance_cm;

    if (control == NULL)
        return false;
    remaining_cm = control->measured_cm - Q3_RETURN_FINAL_TARGET_CM;
    if (remaining_cm < 0.0f)
        remaining_cm = 0.0f;
    speed_cm_s = -control->measured_velocity_cm_s;
    if (speed_cm_s < 0.0f)
        speed_cm_s = 0.0f;
    stopping_distance_cm = speed_cm_s * ((float)Q3_RETURN_FEEDBACK_DELAY_MS / 1000.0f) +
                           (speed_cm_s * speed_cm_s) / (2.0f * Q3_RETURN_DECELERATION_CM_S2) +
                           Q3_RETURN_BRAKE_MARGIN_CM;
    /* 缓慢/噪声大的估计仍会获得一个保守的位置兜底。 */
    return (remaining_cm <= stopping_distance_cm) ||
           (s_return_reference_cm <= Q3_RETURN_BRAKE_REFERENCE_START_CM) ||
           (remaining_cm <= Q3_RETURN_BRAKE_FALLBACK_DISTANCE_CM);
}

static void Enter(Q3SequenceState state, uint32_t now)
{
    s_status.state = state;
    s_status.state_started_ms = now;
    s_result_log_pending = true;
}
void Q3Sequence_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_result_log_pending = false;
    s_return_reference_cm = 0.0f;
    s_return_reference_update_ms = 0U;
    s_plus5_confirm_frames = 0U;
    s_return_zero_retry_ms = 0U;
    s_zero_prepare_retry_ms = (uint32_t)(0U - Q3_ZERO_PREPARE_RETRY_MS);
}
void Q3Sequence_EnableAutoCycle(bool enabled)
{
    /* 手动双按键操作掌控发射；复位时不产生隐式运动。 */
    (void)enabled;
}
bool Q3Sequence_ZeroReference(uint32_t now)
{
    BallActuatorStatus actuator;
    if (Q3Sequence_IsActive())
        return false;
    /* 设计上幂等：第二次调用不得重定义物理
     * 原点。参考仅在显式停止/复位时清除。 */
    if (s_status.zero_reference_ready)
        return true;
    BallActuator_GetStatus(&actuator);
    /* 完成一次 Q3 运行后，终止停止会有意禁用
     * 执行器。因此首次准备重试看到 FOC 关闭是合理的。
     * 唤醒它，恢复正常选项轮询，然后重试直到
     * 反馈证明闭环，再对当前物理姿态置零。 */
    if (!actuator.online || !actuator.firmware_x)
        return false;
    /* 完成的一轮会有意禁用执行器。在本次新的首次按键
     * 零点标记之前，重新建立一个已知的指令会话。 */
    BallActuator_SetCommandSession(true);
    if ((now - s_zero_prepare_retry_ms) < Q3_ZERO_PREPARE_RETRY_MS)
    {
        BallActuator_SetCommandSession(false);
        return false;
    }
    s_zero_prepare_retry_ms = now;
    if (!actuator.foc_closed_loop)
    {
        (void)BallActuator_Enable(true);
        BallActuator_SetCommandSession(false);
        return false;
    }
    if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
        !BallActuator_ZeroAtCurrentPosition())
    {
        BallActuator_SetCommandSession(false);
        return false;
    }
    s_status.zero_reference_ready = true;
    s_zero_ms = now;
    return true;
}
bool Q3Sequence_IsZeroReferenceReady(void)
{
    return s_status.zero_reference_ready;
}
bool Q3Sequence_Start(uint32_t now)
{
    BallActuatorStatus actuator;
    float target = BALL_CONTROL_SINGLE_TARGET_MODE ? BALL_CONTROL_SINGLE_TARGET_CM : 0.0f;
    if (Q3Sequence_IsActive() || !s_status.zero_reference_ready || now - s_zero_ms < 150U)
        return false;
    BallActuator_GetStatus(&actuator);
    /* 成功的 UART 写入并不证明电机已接受零点。 */
    if (!actuator.actual_angle_valid || actuator.last_position_ms == s_zero_ms ||
        (uint32_t)(actuator.last_position_ms - s_zero_ms) > (now - s_zero_ms) ||
        now - actuator.last_position_ms > 200U || fabsf(actuator.actual_angle_deg) > 0.5f)
        return false;
    if (!BallPositionControl_StartTargetCm(target, now))
        return false;
    BallPositionControl_SetOuterProfile(BALL_OUTER_PROFILE_NORMAL);
    s_plus5_confirm_frames = 0U;
    ++s_status.run_sequence;
    s_status.fault = false;
    s_status.time_pass = false;
    s_status.completion_ms = 0U;
    Enter(BALL_CONTROL_SINGLE_TARGET_MODE ? Q3_SEQUENCE_SINGLE_TARGET : Q3_SEQUENCE_CENTER, now);
    return true;
}
void Q3Sequence_Stop(uint32_t now)
{
    BallPositionControl_Stop(now);
    s_status.zero_reference_ready = false;
    /* 新的 PA31 准备指令可在复位或完成运行后立即发送；
     * 后续重试仍受速率限制。 */
    s_zero_prepare_retry_ms = now - Q3_ZERO_PREPARE_RETRY_MS;
    Enter(Q3_SEQUENCE_STOPPED, now);
}
bool Q3Sequence_IsActive(void)
{
    return s_status.state == Q3_SEQUENCE_CENTER || s_status.state == Q3_SEQUENCE_TO_PLUS5 ||
           s_status.state == Q3_SEQUENCE_BRAKE_PLUS5 || s_status.state == Q3_SEQUENCE_TO_MINUS2_5 ||
           s_status.state == Q3_SEQUENCE_TO_MINUS5 || s_status.state == Q3_SEQUENCE_FINAL_HOLD ||
           s_status.state == Q3_SEQUENCE_RETURN_LEVEL ||
           s_status.state == Q3_SEQUENCE_SINGLE_TARGET ||
           s_status.state == Q3_SEQUENCE_RETURN_CRUISE ||
           s_status.state == Q3_SEQUENCE_RETURN_BRAKE;
}
void Q3Sequence_Task(uint32_t now)
{
    BallPositionControlStatus control;
    if (!Q3Sequence_IsActive())
        return;
    BallPositionControl_GetStatus(&control);
    if (BallPositionControl_HasFault())
    {
        s_status.fault = true;
        Enter(Q3_SEQUENCE_FAULT, now);
        return;
    }
    if (s_status.state == Q3_SEQUENCE_CENTER && control.target_reached)
    {
        s_status.motion_started_ms = now;
        BallPositionControl_SetOuterProfile(BALL_OUTER_PROFILE_NORMAL);
        if (BallPositionControl_SetTargetCm(5.0f, now))
        {
            s_plus5_confirm_frames = 0U;
            Enter(Q3_SEQUENCE_TO_PLUS5, now);
        }
    }
    else if (s_status.state == Q3_SEQUENCE_TO_PLUS5)
    {
        /* 同时用位置与速度确认 +5。这可防止单个
         * 延迟/噪声帧就启动返回动作。 */
        if (control.vision_fresh &&
            fabsf(control.measured_cm - 5.0f) <= Q3_PLUS5_SWITCH_TOLERANCE_CM)
        {
            if (s_plus5_confirm_frames < Q3_PLUS5_CONFIRM_FRAMES)
            {
                ++s_plus5_confirm_frames;
            }
            if (s_plus5_confirm_frames >= Q3_PLUS5_CONFIRM_FRAMES)
            {
                s_return_reference_cm = 5.0f;
                s_return_reference_update_ms = now;
                BallPositionControl_SetOuterProfile(BALL_OUTER_PROFILE_RETURN_CRUISE);
                Enter(Q3_SEQUENCE_RETURN_CRUISE, now);
            }
        }
        else if (control.vision_fresh)
        {
            s_plus5_confirm_frames = 0U;
        }
    }
    else if (s_status.state == Q3_SEQUENCE_RETURN_CRUISE)
    {
        if (Q3Sequence_ShouldBrake(&control))
        {
            BallPositionControl_SetOuterProfile(BALL_OUTER_PROFILE_RETURN_BRAKE);
            Enter(Q3_SEQUENCE_RETURN_BRAKE, now);
        }
        else if (!Q3Sequence_AdvanceReturnReference(now))
        {
            s_status.fault = true;
            Enter(Q3_SEQUENCE_FAULT, now);
        }
    }
    else if (s_status.state == Q3_SEQUENCE_RETURN_BRAKE)
    {
        /* 从实际得分带完成，即使最后的 0.1 cm
         * 参考更新被静摩擦卡住。 */
        if (Q3Sequence_FinalPositionAccepted(&control))
        {
            Q3Sequence_RecordFinalCompletion(now);
        }
        else if (!Q3Sequence_AdvanceReturnReference(now))
        {
            s_status.fault = true;
            Enter(Q3_SEQUENCE_FAULT, now);
        }
        else if (s_return_reference_cm <= Q3_RETURN_FINAL_TARGET_CM)
        {
            if (BallPositionControl_SetTargetCm(Q3_RETURN_FINAL_TARGET_CM, now))
            {
                Enter(Q3_SEQUENCE_TO_MINUS5, now);
            }
            else
            {
                s_status.fault = true;
                Enter(Q3_SEQUENCE_FAULT, now);
            }
        }
    }
    else if (s_status.state == Q3_SEQUENCE_TO_MINUS5 && Q3Sequence_FinalPositionAccepted(&control))
    {
        Q3Sequence_RecordFinalCompletion(now);
    }
    else if (s_status.state == Q3_SEQUENCE_FINAL_HOLD &&
             (now - s_status.state_started_ms) >= Q3_FINAL_HOLD_MS)
    {
        /* `0` 是 PA31 确立的会话坐标，而不是 9A/04 或
         * 出厂原点。操作员会在下一次单键试验前
         * 把球放回中心。 */
        if (BallPositionControl_ReturnMotorToZero(now))
        {
            s_return_zero_retry_ms = now;
            Enter(Q3_SEQUENCE_RETURN_LEVEL, now);
        }
        else
        {
            s_status.fault = true;
            Enter(Q3_SEQUENCE_FAULT, now);
        }
    }
    else if (s_status.state == Q3_SEQUENCE_RETURN_LEVEL)
    {
        if (BallPositionControl_IsMotorAtZero())
        {
            /* FC 0 度目标已在 X42S 反馈中观测到。
             * 现在真正暂停位置会话；仅仅进入旧的 READY
             * 状态会让执行器在运行结束后仍保持使能。 */
            BallPositionControl_Stop(now);
            Enter(Q3_SEQUENCE_FINISHED, now);
        }
        else if ((now - s_status.state_started_ms) >= Q3_RETURN_ZERO_TIMEOUT_MS)
        {
            BallPositionControl_Stop(now);
            s_status.fault = true;
            Enter(Q3_SEQUENCE_FAULT, now);
        }
        else if ((now - s_return_zero_retry_ms) >= BALL_CONTROL_ZERO_RETRY_PERIOD_MS)
        {
            /* UART 写入只确认入队，不确认电机运动。持续发送
             * 同一个安全的 0 度指令，直到反馈确认。 */
            if (!BallPositionControl_ReturnMotorToZero(now))
            {
                BallPositionControl_Stop(now);
                s_status.fault = true;
                Enter(Q3_SEQUENCE_FAULT, now);
            }
            else
            {
                s_return_zero_retry_ms = now;
            }
        }
    }
}
bool Q3Sequence_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}
void Q3Sequence_GetStatus(Q3SequenceStatus *status)
{
    if (status)
        *status = s_status;
}
const char *Q3Sequence_StateName(Q3SequenceState state)
{
    switch (state)
    {
    case Q3_SEQUENCE_IDLE:
        return "IDLE";
    case Q3_SEQUENCE_CENTER:
        return "CENTER";
    case Q3_SEQUENCE_SINGLE_TARGET:
        return "TARGET";
    case Q3_SEQUENCE_TO_PLUS5:
        return "TO+5";
    case Q3_SEQUENCE_BRAKE_PLUS5:
        return "BRAKE+5";
    case Q3_SEQUENCE_TO_MINUS2_5:
        return "TO-2.5";
    case Q3_SEQUENCE_TO_MINUS5:
        return "TO-5";
    case Q3_SEQUENCE_RETURN_CRUISE:
        return "RET-CRUISE";
    case Q3_SEQUENCE_RETURN_BRAKE:
        return "RET-BRAKE";
    case Q3_SEQUENCE_FINAL_HOLD:
        return "HOLD-5";
    case Q3_SEQUENCE_RETURN_LEVEL:
        return "RETURN-0";
    case Q3_SEQUENCE_WAIT_CENTER:
        return "READY";
    case Q3_SEQUENCE_FINISHED:
        return "DONE";
    case Q3_SEQUENCE_FAULT:
        return "FAULT";
    case Q3_SEQUENCE_STOPPED:
        return "STOP";
    default:
        return "UNUSED";
    }
}

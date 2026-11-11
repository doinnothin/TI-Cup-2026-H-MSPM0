#include "ball_position_control.h"

#include <string.h>
#include <math.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/ball_control_config.h"
#include "app/q3/q3_control_config.h"
#include "app/q3/ball_state_estimator.h"
#include "code/vision_link.h"

/* 本模块拥有 Q3 序列所用的球位置环。它不
 * 决定比赛轨迹；目标由 q3_sequence.c 提供。 */
static BallPositionControlStatus s_status;
static bool s_have_vision_sequence;
static bool s_have_accepted_vision;
static bool s_have_angle_command;
static bool s_result_log_pending;
static uint8_t s_last_vision_sequence;
static float s_last_command_angle_deg;
static uint32_t s_last_accepted_vision_ms;
static uint32_t s_last_command_ms;
static uint32_t s_run_started_ms;
static uint32_t s_settle_started_ms;
static bool s_have_still_reference;
static uint8_t s_still_reference_sequence;
static float s_still_reference_cm;
static uint32_t s_still_reference_ms;
static float s_kick_angle_deg;
static float s_kick_reached_deg;
static bool s_kick_reached_latched;
static uint32_t s_kick_reached_ms;
static uint32_t s_zero_last_command_ms;
static uint16_t s_last_source_ms;
static float s_dt_s;
static float s_kick_start_cm;
static uint32_t s_tracking_bad_ms;
static BallStateEstimator s_estimator;
static BallOuterPd s_outer_pd;

static bool BallPositionControl_SendAngle(float target_angle_deg, uint32_t now_ms);

static float BallPositionControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float BallPositionControl_Clamp(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static void BallPositionControl_Enter(BallPositionControlState state, uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
    if (state == BALL_CONTROL_KICK)
    {
        s_kick_start_cm = s_status.measured_cm;
        s_status.kick_peak_angle_deg = 0.0f;
        s_status.kick_elapsed_ms = 0U;
        s_kick_reached_latched = false;
        s_kick_reached_ms = 0U;
    }
    if (state == BALL_CONTROL_ENABLE_SETTLE)
    {
        s_zero_last_command_ms = now_ms;
    }
    if ((state == BALL_CONTROL_FAULT) || (state == BALL_CONTROL_STOPPED))
    {
        s_result_log_pending = true;
    }
}

static bool BallPositionControl_Expired(uint32_t now_ms, uint32_t duration_ms)
{
    return (now_ms - s_status.state_started_ms) >= duration_ms;
}

static bool BallPositionControl_ReadVision(uint32_t now_ms, VisionLinkSample *sample)
{
    if ((!VisionLink_GetLatest(sample)) || (!sample->valid))
    {
        return false;
    }

    /* 在此前台调用期间，RX 中断可能为下一个 10 ms 节拍打上时间戳。 */
    /* 这里仅评估最新的数据包。它有意不设置
     * vision_fresh：单帧低置信度/未检测到的数据包会被拒绝，
     * 而最后接受的测量在 BallPositionControl_HasRecentVision() 的
     * 短暂宽限窗口内仍可使用。 */
    return ((int32_t)(now_ms - sample->rx_time_ms) < 0 ||
            (now_ms - sample->rx_time_ms) <= BALL_CONTROL_VISION_TIMEOUT_MS) &&
           sample->confidence_pct >= BALL_CONTROL_CONFIDENCE_MIN && isfinite(sample->position_cm);
}

static bool BallPositionControl_HasRecentVision(uint32_t now_ms)
{
    if (!s_have_accepted_vision)
    {
        s_status.vision_fresh = false;
        s_status.vision_age_ms = 0xFFFFFFFFUL;
        return false;
    }
    s_status.vision_age_ms =
        (int32_t)(now_ms - s_last_accepted_vision_ms) < 0 ? 0U : now_ms - s_last_accepted_vision_ms;
    s_status.vision_fresh = s_status.vision_age_ms <= BALL_CONTROL_VISION_TIMEOUT_MS;
    return s_status.vision_fresh;
}

static bool BallPositionControl_ReadActualAngle(uint32_t now_ms, BallActuatorStatus *actuator)
{
    BallActuator_GetStatus(actuator);
    if (!actuator->actual_angle_valid)
    {
        s_status.actual_angle_fresh = false;
        s_status.actual_angle_age_ms = 0xFFFFFFFFUL;
        return false;
    }

    s_status.actual_angle_deg = actuator->actual_angle_deg;
    if (s_status.state == BALL_CONTROL_KICK)
    {
        if (BallPositionControl_Abs(s_status.actual_angle_deg) >
            BallPositionControl_Abs(s_status.kick_peak_angle_deg))
        {
            s_status.kick_peak_angle_deg = s_status.actual_angle_deg;
        }
        s_status.kick_elapsed_ms = now_ms - s_status.state_started_ms;
    }
    s_status.actual_angle_age_ms = now_ms - actuator->last_position_ms;
    s_status.actual_angle_fresh = s_status.actual_angle_age_ms <= BALL_CONTROL_ANGLE_TIMEOUT_MS;
    return s_status.actual_angle_fresh;
}

static void BallPositionControl_Fail(BallPositionControlFault fault, uint32_t now_ms)
{
    s_status.fault = fault;
    s_status.control_output_deg = 0.0f;
    s_status.target_angle_deg = 0.0f;
    (void)BallActuator_Stop();
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    BallPositionControl_Enter(BALL_CONTROL_FAULT, now_ms);
}

/* 持续的摄像头/链路丢失与电机故障不同。X42S
 * 角度伺服仍然正常，因此向已建立的会话下发零位指令并
 * 保持其使能，而不是冻结最后斜率或突然禁用。 */
static void BallPositionControl_FailReturnLevel(BallPositionControlFault fault, uint32_t now_ms)
{
    s_status.fault = fault;
    s_status.control_output_deg = 0.0f;
    if (!BallPositionControl_SendAngle(0.0f, now_ms))
    {
        BallPositionControl_Fail(BALL_CONTROL_FAULT_POSITION_COMMAND, now_ms);
        return;
    }
    BallOuterPd_ResetTrajectory(&s_outer_pd);
    BallPositionControl_Enter(BALL_CONTROL_FAULT, now_ms);
}

static bool BallPositionControl_SendAngle(float target_angle_deg, uint32_t now_ms)
{
    if (!isfinite(target_angle_deg) ||
        BallPositionControl_Abs(target_angle_deg) > Q3_MOTOR_SAFE_LIMIT_DEG)
        return false;
    if (!BallActuator_SetAbsoluteAngleDeg(target_angle_deg))
    {
        return false;
    }
    s_status.target_angle_deg = target_angle_deg;
    s_last_command_angle_deg = target_angle_deg;
    s_last_command_ms = now_ms;
    s_have_angle_command = true;
    ++s_status.command_count;
    return true;
}

static void BallPositionControl_SelectKick(float error_cm)
{
    if (error_cm >= 0.0f)
    {
        s_kick_angle_deg = BALL_CONTROL_BREAKAWAY_TO_VISUAL_POS_DEG;
        s_kick_reached_deg = BALL_CONTROL_BREAKAWAY_POS_REACHED_DEG;
    }
    else
    {
        s_kick_angle_deg = BALL_CONTROL_BREAKAWAY_TO_VISUAL_NEG_DEG;
        s_kick_reached_deg = BALL_CONTROL_BREAKAWAY_NEG_REACHED_DEG;
    }
}

static bool BallPositionControl_UpdateMeasurement(const VisionLinkSample *sample, uint32_t now_ms)
{
    uint32_t dt_ms;

    (void)now_ms;

    if (s_estimator.initialized)
    {
        dt_ms = (uint16_t)(sample->source_timestamp_ms - s_last_source_ms);
        if ((dt_ms > 0U) && (dt_ms <= BALL_CONTROL_SAMPLE_MAX_MS))
        {
            s_dt_s = (float)dt_ms / 1000.0f;
        }
        /* 复位/卡住的源时钟并不能证明球已停下。 */
        else
        {
            return false;
        }
    }
    else
    {
        s_dt_s = 0.0f;
    }

    if (!BallStateEstimator_Update(&s_estimator, sample->position_cm, s_dt_s))
    {
        s_status.estimator_innovation_cm = s_estimator.innovation_cm;
        s_status.estimator_reject_count = s_estimator.rejected_count;
        return false;
    }

    s_status.vision_sequence = sample->sequence;
    s_status.measured_cm = s_estimator.position_cm;
    s_status.measured_velocity_cm_s = s_estimator.velocity_cm_s;
    s_status.vision_velocity_cm_s = sample->velocity_cm_s;
    s_status.error_cm = s_status.target_cm - s_status.measured_cm;
    s_status.estimator_innovation_cm = s_estimator.innovation_cm;
    s_status.estimator_reject_count = s_estimator.rejected_count;
    s_last_source_ms = sample->source_timestamp_ms;
    s_status.sample_dt_ms = (uint32_t)(s_dt_s * 1000.0f + 0.5f);
    return true;
}

static bool BallPositionControl_HasNewVision(const VisionLinkSample *sample)
{
    if ((!s_have_vision_sequence) || (sample->sequence != s_last_vision_sequence))
    {
        s_have_vision_sequence = true;
        s_last_vision_sequence = sample->sequence;
        return true;
    }
    return false;
}

static void BallPositionControl_ResetStillness(void)
{
    s_have_still_reference = false;
    s_still_reference_sequence = 0U;
    s_still_reference_cm = 0.0f;
    s_still_reference_ms = 0U;
}

/* 真正的静止球判定必须基于一段时间窗口内的位置，而不是
 * 两帧摄像头的数值微分。由于像素量化，
 * 20 ms 内 0.1 cm 的位置跳变即使对静止球也会表现为 5 cm/s。 */
static bool BallPositionControl_PositionStill(uint32_t now_ms)
{
    if (!s_have_still_reference)
    {
        s_have_still_reference = true;
        s_still_reference_sequence = s_status.vision_sequence;
        s_still_reference_cm = s_status.measured_cm;
        s_still_reference_ms = now_ms;
        return false;
    }

    if (s_status.vision_sequence != s_still_reference_sequence)
    {
        s_still_reference_sequence = s_status.vision_sequence;
        if (BallPositionControl_Abs(s_status.measured_cm - s_still_reference_cm) >
            BALL_CONTROL_STILL_POSITION_BAND_CM)
        {
            s_still_reference_cm = s_status.measured_cm;
            s_still_reference_ms = now_ms;
        }
    }

    return (now_ms - s_still_reference_ms) >= BALL_CONTROL_STILL_WINDOW_MS;
}

static bool BallPositionControl_WithinTarget(uint32_t now_ms)
{
    return BallPositionControl_Abs(s_status.error_cm) <= BALL_CONTROL_TARGET_TOLERANCE_CM &&
           BallPositionControl_Abs(s_status.measured_velocity_cm_s) <=
               BALL_CONTROL_SETTLE_SPEED_CM_S &&
           BallPositionControl_PositionStill(now_ms);
}

static void BallPositionControl_RunOuterPd(const VisionLinkSample *sample, uint32_t now_ms)
{
    BallOuterPdOutput output;
    float command;
    (void)sample;
    if (s_dt_s <= 0.0f)
        return;
    if (!BallOuterPd_Update(&s_outer_pd,
                            s_status.error_cm,
                            s_status.measured_velocity_cm_s,
                            s_dt_s,
                            s_status.actual_angle_deg,
                            &output))
    {
        BallPositionControl_Fail(BALL_CONTROL_FAULT_POSITION_COMMAND, now_ms);
        return;
    }
    ++s_status.pid_updates;
    s_status.dispatch_age_ms = s_status.vision_age_ms;
    s_status.outer_pd = output;
    s_status.braking = output.braking;
    s_status.stopping_distance_cm = output.predicted_stop_cm;
    s_status.pid_p_deg = output.p_deg;
    s_status.pid_i_deg = 0.0f;
    s_status.pid_d_deg = output.d_deg;
    command = output.motor_deg;
    s_status.control_output_deg = command;
    if (!s_have_angle_command ||
        (fabsf(command - s_last_command_angle_deg) >= BALL_CONTROL_COMMAND_DELTA_DEG &&
         now_ms - s_last_command_ms >= BALL_CONTROL_COMMAND_MIN_PERIOD_MS))
    {
        if (!BallPositionControl_SendAngle(command, now_ms))
            BallPositionControl_Fail(BALL_CONTROL_FAULT_POSITION_COMMAND, now_ms);
    }
}

void BallPositionControl_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = BALL_CONTROL_IDLE;
    s_status.target_cm = 0.0f;
    BallStateEstimator_Init(&s_estimator);
    BallOuterPd_Init(&s_outer_pd);
}

bool BallPositionControl_Start(uint32_t now_ms)
{
    if (BallPositionControl_IsActive())
    {
        return false;
    }

    ++s_status.run_sequence;
    s_status.fault = BALL_CONTROL_FAULT_NONE;
    s_status.target_cm = 0.0f;
    s_status.measured_cm = 0.0f;
    s_status.error_cm = 0.0f;
    s_status.control_output_deg = 0.0f;
    s_status.target_angle_deg = 0.0f;
    s_status.actual_angle_deg = 0.0f;
    s_status.measured_velocity_cm_s = 0.0f;
    s_status.vision_velocity_cm_s = 0.0f;
    s_status.pid_p_deg = 0.0f;
    s_status.pid_i_deg = 0.0f;
    s_status.pid_d_deg = 0.0f;
    memset(&s_status.outer_pd, 0, sizeof(s_status.outer_pd));
    s_status.estimator_innovation_cm = 0.0f;
    s_status.estimator_reject_count = 0U;
    s_status.target_reached = false;
    s_status.rekick_count = 0U;
    s_status.pid_updates = 0U;
    s_status.command_count = 0U;
    s_status.braking = false;
    s_status.stopping_distance_cm = 0.0f;
    s_dt_s = 0.0f;
    s_tracking_bad_ms = 0U;
    s_have_vision_sequence = false;
    s_have_accepted_vision = false;
    s_have_angle_command = false;
    s_last_vision_sequence = 0U;
    s_last_command_angle_deg = 0.0f;
    s_last_accepted_vision_ms = 0U;
    s_last_command_ms = 0U;
    s_settle_started_ms = 0U;
    BallPositionControl_ResetStillness();
    s_result_log_pending = false;
    s_run_started_ms = now_ms;
    s_kick_angle_deg = BALL_CONTROL_BREAKAWAY_TO_VISUAL_POS_DEG;
    s_kick_reached_deg = BALL_CONTROL_BREAKAWAY_POS_REACHED_DEG;
    BallStateEstimator_Init(&s_estimator);
    BallOuterPd_Init(&s_outer_pd);
    BallActuator_SetCommandSession(true);
    BallPositionControl_Enter(BALL_CONTROL_PRECHECK, now_ms);
    return true;
}

bool BallPositionControl_StartTargetCm(float target_cm, uint32_t now_ms)
{
    if (!isfinite(target_cm) || fabsf(target_cm) > BALL_CONTROL_TARGET_LIMIT_CM)
        return false;
    if (!BallPositionControl_Start(now_ms))
        return false;
    return BallPositionControl_SetTargetCm(target_cm, now_ms);
}

bool BallPositionControl_SetTargetCm(float target_cm, uint32_t now_ms)
{
    float error_cm;
    bool target_changed;

    if (!BallPositionControl_IsActive() || !isfinite(target_cm) ||
        (BallPositionControl_Abs(target_cm) > BALL_CONTROL_TARGET_LIMIT_CM))
    {
        return false;
    }

    target_changed = (BallPositionControl_Abs(target_cm - s_status.target_cm) > 0.01f);
    s_status.target_cm = target_cm;
    s_status.target_reached = false;
    /* 仅在真实的目标切换时复位。同一目标的再次冲击必须
     * 消耗有限预算，否则 KICK 会无限重复。 */
    if (target_changed)
    {
        s_status.rekick_count = 0U;
        /* 平滑的 Q3 返回每秒多次更新此参考值。
         * 保留最初运行的截止时间；否则每个参考步进
         * 都会无限推迟看门狗。 */
    }
    s_settle_started_ms = 0U;
    BallPositionControl_ResetStillness();
    error_cm = target_cm - s_status.measured_cm;
    BallPositionControl_SelectKick(error_cm);

    /* 到达后的目标变化是一次新的物理动作。复用已
     * 使能的执行器，但以测量方向上的冲击重新开始。 */
    if ((s_status.state == BALL_CONTROL_ACTIVE) || (s_status.state == BALL_CONTROL_HOLD))
    {
        /* 在反向时保留测量速度。只有已证实的静止
         * 停滞才可请求另一次冲击；目标步进本身不得请求。 */
        BallPositionControl_Enter(BALL_CONTROL_ACTIVE, now_ms);
    }
    return true;
}

void BallPositionControl_SetOuterProfile(BallOuterPdProfile profile)
{
    BallOuterPd_SetProfile(&s_outer_pd, profile);
}

bool BallPositionControl_ReturnMotorToZero(uint32_t now_ms)
{
    /* 初始的 FC 0 指令把本控制器切换到 STOPPED，使其
     * 外环 PD 无法与返回相抗衡。允许序列在等待
     * 真实 X42S 位置反馈期间重新下发同一目标。 */
    if (!BallPositionControl_IsActive() && (s_status.state != BALL_CONTROL_STOPPED))
    {
        return false;
    }

    /* FC 目标零使用本会话的坐标原点与轨迹
     * 限幅。9A/04 也是回到坐标原点的回零操作；
     * 二者都不是单独存储的 93 单圈原点标记。 */
    if (!BallPositionControl_SendAngle(0.0f, now_ms))
    {
        return false;
    }
    s_status.control_output_deg = 0.0f;
    s_status.target_reached = false;
    BallPositionControl_Enter(BALL_CONTROL_STOPPED, now_ms);
    return true;
}

bool BallPositionControl_IsMotorAtZero(void)
{
    BallActuatorStatus actuator;

    BallActuator_GetStatus(&actuator);
    return actuator.actual_angle_valid && (BallPositionControl_Abs(actuator.actual_angle_deg) <=
                                           BALL_CONTROL_RETURN_TOLERANCE_DEG);
}

void BallPositionControl_Stop(uint32_t now_ms)
{
    if (s_status.state != BALL_CONTROL_IDLE)
    {
        (void)BallActuator_Stop();
        (void)BallActuator_Enable(false);
        BallActuator_SetCommandSession(false);
        s_status.target_angle_deg = 0.0f;
        s_status.control_output_deg = 0.0f;
        BallPositionControl_Enter(BALL_CONTROL_STOPPED, now_ms);
    }
}

bool BallPositionControl_IsActive(void)
{
    return (s_status.state >= BALL_CONTROL_PRECHECK) && (s_status.state <= BALL_CONTROL_HOLD);
}

bool BallPositionControl_IsArrived(void)
{
    return s_status.target_reached;
}

bool BallPositionControl_HasFault(void)
{
    return s_status.state == BALL_CONTROL_FAULT;
}

bool BallPositionControl_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void BallPositionControl_Task(uint32_t now_ms)
{
    BallActuatorStatus actuator;
    VisionLinkSample sample;
    bool usable_newest_vision;
    bool fresh_vision;
    bool fresh_angle;
    bool new_sample = false;

    if (!BallPositionControl_IsActive())
    {
        return;
    }

    usable_newest_vision = BallPositionControl_ReadVision(now_ms, &sample);
    fresh_angle = BallPositionControl_ReadActualAngle(now_ms, &actuator);
    if (usable_newest_vision && BallPositionControl_HasNewVision(&sample))
    {
        if (!BallPositionControl_UpdateMeasurement(&sample, now_ms))
        {
            /* 拒绝一个损坏/非单调的源时间帧。不允许其年龄
             * 刷新看门狗，因此持续的问题
             * 仍会在下方触发 VISION_STALE。 */
            new_sample = false;
        }
        else
        {
            s_have_accepted_vision = true;
            s_last_accepted_vision_ms = sample.rx_time_ms;
            new_sample = true;
        }
    }
    fresh_vision = BallPositionControl_HasRecentVision(now_ms);
    /* 缓存数据可以维持看门狗，但无法证明已到达。 */
    s_status.vision_fresh = fresh_vision && new_sample;
    /* 在每一个运动阶段（包括冲击）都检查最新坐标。 */
    if (usable_newest_vision && fabsf(sample.position_cm) >= BALL_CONTROL_SAFETY_POSITION_CM)
    {
        BallPositionControl_Fail(BALL_CONTROL_FAULT_SAFETY_POSITION, now_ms);
        return;
    }
    if (s_have_angle_command && fresh_angle &&
        fabsf(s_last_command_angle_deg - actuator.actual_angle_deg) > 2.0f)
    {
        if (s_tracking_bad_ms == 0U)
            s_tracking_bad_ms = now_ms;
        else if (now_ms - s_tracking_bad_ms > 1000U)
        {
            BallPositionControl_Fail(BALL_CONTROL_FAULT_TRACKING, now_ms);
            return;
        }
    }
    else
        s_tracking_bad_ms = 0U;

    if ((!fresh_vision) || (!fresh_angle))
    {
        if (!fresh_vision)
        {
            BallPositionControl_FailReturnLevel(BALL_CONTROL_FAULT_VISION_STALE, now_ms);
        }
        else
        {
            BallPositionControl_Fail(BALL_CONTROL_FAULT_ANGLE_STALE, now_ms);
        }
        return;
    }

    if (!usable_newest_vision)
    {
        s_status.target_reached = false;
        s_settle_started_ms = 0U;
        BallPositionControl_ResetStillness();
        if (s_status.state == BALL_CONTROL_ACTIVE || s_status.state == BALL_CONTROL_HOLD)
            return;
    }

    switch (s_status.state)
    {
    case BALL_CONTROL_PRECHECK:
        if (!actuator.online || !actuator.firmware_x || !actuator.foc_closed_loop ||
            !fresh_vision || !fresh_angle)
        {
            BallPositionControl_Fail(!fresh_vision ? BALL_CONTROL_FAULT_VISION_STALE
                                                   : BALL_CONTROL_FAULT_ACTUATOR_NOT_READY,
                                     now_ms);
        }
        else if (!BallActuator_Enable(true))
        {
            BallPositionControl_Fail(BALL_CONTROL_FAULT_ENABLE_COMMAND, now_ms);
        }
        else
        {
            BallPositionControl_SelectKick(s_status.target_cm - sample.position_cm);
            /* PA31 已确立水平位置作为坐标零点（0A 6D）。
                 * 直接开始正常 FC 跟踪，无需单独的回零动作。 */
            if (!BallActuator_PrepareAbsolutePosition(BALL_CONTROL_ACCEL_RPM_S,
                                                      BALL_CONTROL_DECEL_RPM_S,
                                                      BALL_CONTROL_MAX_SPEED_RPM,
                                                      BALL_CONTROL_MAX_CURRENT_MA))
            {
                BallPositionControl_Fail(BALL_CONTROL_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                BallOuterPd_ResetTrajectory(&s_outer_pd);
                BallPositionControl_Enter(BALL_CONTROL_NORMAL_PARAM_GAP, now_ms);
            }
        }
        break;

    case BALL_CONTROL_ENABLE_SETTLE:
        if (BallPositionControl_Expired(now_ms, BALL_CONTROL_ENABLE_SETTLE_MS))
        {
            if (BallPositionControl_Abs(s_status.actual_angle_deg) >
                BALL_CONTROL_RETURN_TOLERANCE_DEG)
            {
                if (BallPositionControl_Expired(now_ms, BALL_CONTROL_ZERO_MOVE_TIMEOUT_MS))
                {
                    BallPositionControl_Fail(BALL_CONTROL_FAULT_ZERO_COMMAND, now_ms);
                }
                break;
            }
            if (!BallActuator_ZeroAtCurrentPosition())
            {
                BallPositionControl_Fail(BALL_CONTROL_FAULT_ZERO_COMMAND, now_ms);
            }
            else
            {
                BallPositionControl_Enter(BALL_CONTROL_ZERO_GAP, now_ms);
            }
        }
        break;

    case BALL_CONTROL_ZERO_GAP:
        if (BallPositionControl_Expired(now_ms, BALL_CONTROL_COMMAND_GAP_MS))
        {
            if (!BallActuator_PrepareAbsolutePosition(BALL_CONTROL_BREAKAWAY_ACCEL_RPM_S,
                                                      BALL_CONTROL_BREAKAWAY_DECEL_RPM_S,
                                                      BALL_CONTROL_BREAKAWAY_SPEED_RPM,
                                                      BALL_CONTROL_BREAKAWAY_MAX_CURRENT_MA))
            {
                BallPositionControl_Fail(BALL_CONTROL_FAULT_PARAMETER_COMMAND, now_ms);
            }
            else
            {
                BallPositionControl_Enter(BALL_CONTROL_KICK_PARAM_GAP, now_ms);
            }
        }
        break;

    case BALL_CONTROL_KICK_PARAM_GAP:
        if (BallPositionControl_Expired(now_ms, BALL_CONTROL_COMMAND_GAP_MS))
        {
            if (!BallPositionControl_SendAngle(s_kick_angle_deg, now_ms))
            {
                BallPositionControl_Fail(BALL_CONTROL_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                BallPositionControl_Enter(BALL_CONTROL_KICK, now_ms);
            }
        }
        break;

    case BALL_CONTROL_KICK:
        if (!s_kick_reached_latched &&
            (((s_kick_angle_deg < 0.0f) && (s_status.actual_angle_deg <= s_kick_reached_deg)) ||
             ((s_kick_angle_deg > 0.0f) && (s_status.actual_angle_deg >= s_kick_reached_deg))))
        {
            s_kick_reached_latched = true;
            s_kick_reached_ms = now_ms;
        }
        if (fabsf(s_status.measured_cm - s_kick_start_cm) >= BALL_CONTROL_KICK_MOVE_CM ||
            fabsf(s_status.error_cm) <= BALL_CONTROL_TARGET_TOLERANCE_CM ||
            (s_kick_reached_latched &&
             ((now_ms - s_kick_reached_ms) >= BALL_CONTROL_BREAKAWAY_HOLD_MS)))
        {
            /* 在 F1 切换到正常跟踪限幅期间保持启动斜率。
                 * 之后下一个新的视觉帧便从冲击角度直接驱动
                 * PID；此处强制 0 度过去会造成
                 * 开环返回以及数百毫秒的
                 * 控制空档。 */
            if (!BallActuator_PrepareAbsolutePosition(BALL_CONTROL_ACCEL_RPM_S,
                                                      BALL_CONTROL_DECEL_RPM_S,
                                                      BALL_CONTROL_MAX_SPEED_RPM,
                                                      BALL_CONTROL_MAX_CURRENT_MA))
            {
                BallPositionControl_Fail(BALL_CONTROL_FAULT_PARAMETER_COMMAND, now_ms);
            }
            else
            {
                BallOuterPd_ResetTrajectory(&s_outer_pd);
                BallPositionControl_Enter(BALL_CONTROL_NORMAL_PARAM_GAP, now_ms);
            }
        }
        else if (BallPositionControl_Expired(now_ms, BALL_CONTROL_BREAKAWAY_TIMEOUT_MS))
        {
            BallPositionControl_Fail(BALL_CONTROL_FAULT_KICK_TIMEOUT, now_ms);
        }
        break;

    case BALL_CONTROL_RETURN:
        if (BallPositionControl_Abs(s_status.actual_angle_deg) <= BALL_CONTROL_RETURN_TOLERANCE_DEG)
        {
            if (!BallActuator_PrepareAbsolutePosition(BALL_CONTROL_ACCEL_RPM_S,
                                                      BALL_CONTROL_DECEL_RPM_S,
                                                      BALL_CONTROL_MAX_SPEED_RPM,
                                                      BALL_CONTROL_MAX_CURRENT_MA))
            {
                BallPositionControl_Fail(BALL_CONTROL_FAULT_PARAMETER_COMMAND, now_ms);
            }
            else
            {
                BallPositionControl_Enter(BALL_CONTROL_NORMAL_PARAM_GAP, now_ms);
            }
        }
        else if (BallPositionControl_Expired(now_ms, BALL_CONTROL_RETURN_TIMEOUT_MS))
        {
            BallPositionControl_Fail(BALL_CONTROL_FAULT_RETURN_TIMEOUT, now_ms);
        }
        break;

    case BALL_CONTROL_NORMAL_PARAM_GAP:
        if (BallPositionControl_Expired(now_ms, BALL_CONTROL_NORMAL_PARAM_GAP_MS))
        {
            BallPositionControl_ResetStillness();
            BallPositionControl_Enter(BALL_CONTROL_ACTIVE, now_ms);
        }
        break;

    case BALL_CONTROL_ACTIVE:
    case BALL_CONTROL_HOLD:
        if (BallPositionControl_Abs(s_status.measured_cm) > BALL_CONTROL_SAFETY_POSITION_CM)
        {
            BallPositionControl_Fail(BALL_CONTROL_FAULT_SAFETY_POSITION, now_ms);
            break;
        }
        if ((s_status.state == BALL_CONTROL_ACTIVE) &&
            ((now_ms - s_run_started_ms) >= BALL_CONTROL_RUN_TIMEOUT_MS))
        {
            BallPositionControl_Fail(BALL_CONTROL_FAULT_RUN_TIMEOUT, now_ms);
            break;
        }

        if (!new_sample)
            break;
        BallPositionControl_RunOuterPd(&sample, now_ms);
        if (s_status.state == BALL_CONTROL_FAULT)
        {
            break;
        }
        if (BallPositionControl_WithinTarget(now_ms))
        {
            if (s_settle_started_ms == 0U)
            {
                s_settle_started_ms = now_ms;
            }
            else if ((now_ms - s_settle_started_ms) >= BALL_CONTROL_SETTLE_HOLD_MS)
            {
                s_status.target_reached = true;
                BallPositionControl_Enter(BALL_CONTROL_HOLD, now_ms);
            }
        }
        else
        {
            s_settle_started_ms = 0U;
            s_status.target_reached = false;
            if (s_status.state == BALL_CONTROL_HOLD &&
                fabsf(s_status.error_cm) > BALL_CONTROL_TARGET_TOLERANCE_CM)
            {
                s_run_started_ms = now_ms;
                BallPositionControl_Enter(BALL_CONTROL_ACTIVE, now_ms);
            }
        }

        if ((s_status.state == BALL_CONTROL_ACTIVE) && !s_status.outer_pd.stiction_assist &&
            (BallPositionControl_Abs(s_status.error_cm) > BALL_CONTROL_REKICK_ERROR_CM) &&
            BallPositionControl_PositionStill(now_ms))
        {
            if (s_status.rekick_count < BALL_CONTROL_REKICK_MAX_COUNT)
            {
                ++s_status.rekick_count;
                BallPositionControl_SelectKick(s_status.error_cm);
                if (!BallActuator_PrepareAbsolutePosition(BALL_CONTROL_BREAKAWAY_ACCEL_RPM_S,
                                                          BALL_CONTROL_BREAKAWAY_DECEL_RPM_S,
                                                          BALL_CONTROL_BREAKAWAY_SPEED_RPM,
                                                          BALL_CONTROL_BREAKAWAY_MAX_CURRENT_MA))
                    BallPositionControl_Fail(BALL_CONTROL_FAULT_PARAMETER_COMMAND, now_ms);
                else
                    BallPositionControl_Enter(BALL_CONTROL_KICK_PARAM_GAP, now_ms);
            }
            else
            {
                BallPositionControl_Fail(BALL_CONTROL_FAULT_RUN_TIMEOUT, now_ms);
            }
        }
        break;

    default:
        BallPositionControl_Fail(BALL_CONTROL_FAULT_POSITION_COMMAND, now_ms);
        break;
    }
}

void BallPositionControl_GetStatus(BallPositionControlStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

const char *BallPositionControl_StateName(BallPositionControlState state)
{
    switch (state)
    {
    case BALL_CONTROL_IDLE:
        return "IDLE";
    case BALL_CONTROL_PRECHECK:
        return "CHECK";
    case BALL_CONTROL_ENABLE_SETTLE:
        return "ENABLE";
    case BALL_CONTROL_ZERO_GAP:
        return "ZERO";
    case BALL_CONTROL_KICK_PARAM_GAP:
        return "KPARAM";
    case BALL_CONTROL_KICK:
        return "KICK";
    case BALL_CONTROL_RETURN:
        return "RETURN";
    case BALL_CONTROL_NORMAL_PARAM_GAP:
        return "PPARAM";
    case BALL_CONTROL_ACTIVE:
        return "P";
    case BALL_CONTROL_HOLD:
        return "HOLD";
    case BALL_CONTROL_FAULT:
        return "FAULT";
    case BALL_CONTROL_STOPPED:
        return "STOP";
    default:
        return "UNKNOWN";
    }
}

#include "q4_static_recovery.h"

#if Q4_STATIC_RECOVERY_TEST_ENABLED

#include <math.h>
#include <string.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/ball_control_config.h"
#include "app/q3/ball_outer_pd.h"
#include "app/q3/ball_state_estimator.h"
#include "app/q3/q3_control_config.h"
#include "app/q3/pendulum_geometry.h"
#include "q4_control_config.h"
#include "code/vision_link.h"

static Q4StaticStatus s_status;
static bool s_result_log_pending;
static uint16_t s_random_state;
static uint8_t s_disturb_stage;
static uint32_t s_disturb_started_ms;
static uint8_t s_zero_stage;
static uint32_t s_zero_started_ms;
static uint32_t s_zero_last_command_ms;
static bool s_zero_nudge_sent;
static BallOuterPd s_recovery_pd;
static BallStateEstimator s_recovery_estimator;
static uint32_t s_last_vision_rx_ms;
static uint32_t s_last_valid_vision_ms;
static bool s_have_recovery_sample;
static uint32_t s_hold_started_ms;
static uint32_t s_hold_candidate_ms;
static float s_last_commanded_angle_deg;
static uint32_t s_recovery_command_ready_ms;
static uint32_t s_last_command_send_ms;
static bool s_recovery_breakaway_active;
static bool s_direct_recovery_active;
static bool s_startup_zero_pending;
static uint8_t s_recovery_breakaway_count;
static uint32_t s_recovery_still_started_ms;
static uint32_t s_recovery_breakaway_started_ms;
static uint32_t s_recovery_started_ms;
static uint32_t s_ready_center_since_ms;
static bool s_fault_zero_pending;
static float s_recovery_breakaway_limit_deg;

static void Q4StaticRecovery_Enter(Q4StaticState state, uint32_t now_ms);
static float Q4StaticRecovery_Abs(float value);
static bool Q4StaticRecovery_ComputeFastMotor(float error_cm,
                                              float velocity_cm_s,
                                              BallOuterPdProfile profile,
                                              float *motor_deg);
static void Q4StaticRecovery_ApplyMeasuredDistanceLimit(float measured_cm,
                                                        BallOuterPdProfile profile,
                                                        float *motor_deg);

/* 选用已为 Q3 调好的同样三种输出 profile。Q4 保持
 * 零厘米参考，因此把 Q3 的返回距离判据表述为
 * 绝对距离与朝零方向的速度。不修改任何 Q3 源码或参数；
 * 这里仅选择 Q4 封装层使用哪一个冻结的 profile。 */
static BallOuterPdProfile Q4StaticRecovery_SelectProfile(void)
{
    float distance_cm = Q4StaticRecovery_Abs(s_recovery_estimator.position_cm);
    float speed_toward_cm_s = 0.0f;
    float speed_abs = Q4StaticRecovery_Abs(s_recovery_estimator.velocity_cm_s);
    float stopping_distance_cm;
    float brake_fallback_cm = Q4_RECOVERY_BRAKE_FALLBACK_CM;

    /* 两个机械方向并不对称。对受益于较早
     * 7 cm 制动进入的方向保留该值，而相反的误差方向
     * 继续使用已验证的 5 cm 进入值，直到对其
     * 独立调参。 */
    if (s_recovery_estimator.position_cm > 0.0f)
    {
        brake_fallback_cm = Q4_RECOVERY_BRAKE_FALLBACK_CM_ERROR_NEG;
    }

    /* 扰动后的第一个视觉采样可能仍是球尚未明显移动之前
     * 延迟的中心采样。对该第一个采样保持有限的
     * Q4 起步脉冲；否则调用方会在目标发出之前
     * 就将其清除。 */
    if (s_recovery_breakaway_active && !s_have_recovery_sample)
    {
        return BALL_OUTER_PROFILE_NORMAL;
    }

    if ((s_recovery_estimator.position_cm * s_recovery_estimator.velocity_cm_s) < 0.0f)
    {
        speed_toward_cm_s = speed_abs;
    }
    stopping_distance_cm =
        speed_toward_cm_s * ((float)Q3_RETURN_FEEDBACK_DELAY_MS / 1000.0f) +
        (speed_toward_cm_s * speed_toward_cm_s) / (2.0f * Q3_RETURN_DECELERATION_CM_S2) +
        Q4_RECOVERY_BRAKE_MARGIN_CM;

    /* 静止的球即使处于正常制动距离内，仍可能被困在
     * 实测的机械死区中。此时让有限的 Q4 起步脉冲
     * 优先于 BRAKE；否则调用方会在脉冲发出之前
     * 就将其清除。 */
    if (s_recovery_breakaway_active && (speed_abs <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S) &&
        (distance_cm > BALL_OUTER_STICTION_ERROR_CM))
    {
        return BALL_OUTER_PROFILE_NORMAL;
    }

    if ((distance_cm <= Q4_HOLD_POSITION_TOLERANCE_CM) && (speed_abs <= Q4_HOLD_SPEED_LIMIT_CM_S))
    {
        return BALL_OUTER_PROFILE_RETURN_HOLD;
    }
    if ((distance_cm <= stopping_distance_cm) || (distance_cm <= brake_fallback_cm))
    {
        return BALL_OUTER_PROFILE_RETURN_BRAKE;
    }
    /* 仅在球仍有效静止时保留 Q3 的一次性静摩擦辅助。
     * 一旦测得运动，立即退出 NORMAL，
     * 使该辅助不会变成持续驱动。 */
    if (s_recovery_breakaway_active && (speed_abs <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S))
    {
        return BALL_OUTER_PROFILE_NORMAL;
    }
    if (speed_abs > BALL_OUTER_STICTION_RELEASE_SPEED_CM_S)
    {
        return BALL_OUTER_PROFILE_RETURN_CRUISE;
    }
    return BALL_OUTER_PROFILE_RETURN_CRUISE;
}

static void Q4StaticRecovery_ApplyMeasuredDistanceLimit(float measured_cm,
                                                        BallOuterPdProfile profile,
                                                        float *motor_deg)
{
    float mid_motor_deg;
    if ((motor_deg == NULL) || (profile == BALL_OUTER_PROFILE_RETURN_HOLD) ||
        (Q4StaticRecovery_Abs(measured_cm) > Q4_RECOVERY_MID_DISTANCE_CM))
    {
        return;
    }
    if (!PendulumGeometry_BeamDegToMotorDeg(Q4_RECOVERY_MID_BEAM_LIMIT_DEG, &mid_motor_deg))
    {
        return;
    }
    if (Q4StaticRecovery_Abs(*motor_deg) > Q4StaticRecovery_Abs(mid_motor_deg))
    {
        *motor_deg = (*motor_deg < 0.0f) ? -mid_motor_deg : mid_motor_deg;
    }
}

static bool Q4StaticRecovery_BeginZero(uint32_t now_ms)
{
    BallActuatorStatus actuator;

    BallActuator_GetStatus(&actuator);
    if (!actuator.online || !actuator.options_valid || !actuator.firmware_x ||
        !actuator.foc_closed_loop || !actuator.actual_angle_valid)
    {
        return false;
    }
    BallActuator_SetCommandSession(true);
    if (!BallActuator_ClearProtection() || !BallActuator_Enable(true))
    {
        BallActuator_SetCommandSession(false);
        return false;
    }
    s_zero_stage = 1U;
    s_zero_started_ms = 0U;
    s_zero_last_command_ms = 0U;
    s_zero_nudge_sent = false;
    Q4StaticRecovery_Enter(Q4_STATIC_RETURN_ZERO, now_ms);
    return true;
}

static void Q4StaticRecovery_Fault(uint32_t now_ms)
{
    /* 先停止，再尝试将 Q4 会话坐标归零，然后才
     * 暴露终态 FAULT。此逻辑仅用于 Q4；Q3 的故障路径
     * 保持不变。归零超时仍是硬性终态故障。 */
    (void)BallActuator_Stop();
    s_status.fault = true;
    s_fault_zero_pending = true;
    if ((s_status.state != Q4_STATIC_RETURN_ZERO) && Q4StaticRecovery_BeginZero(now_ms))
    {
        return;
    }
    if (s_status.fault_reason == 0U)
    {
        s_status.fault_reason = 6U;
    }
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    s_fault_zero_pending = false;
    Q4StaticRecovery_Enter(Q4_STATIC_FAULT, now_ms);
}

static bool Q4StaticRecovery_UpdateControl(uint32_t now_ms)
{
    VisionLinkSample sample;
    BallActuatorStatus actuator;
    BallOuterPdOutput output;
    BallOuterPdProfile profile;
    float control_error_cm;
    float dt_s;

    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm) ||
        !isfinite(sample.velocity_cm_s) || sample.confidence_pct < Q4_RECOVERY_CONFIDENCE_MIN ||
        (now_ms - sample.rx_time_ms) > Q4_VISUAL_READY_AGE_MS)
    {
        if (s_have_recovery_sample && (now_ms - s_last_valid_vision_ms) <= Q4_VISUAL_READY_AGE_MS)
        {
            return true;
        }
        s_status.fault_reason = 1U;
        return false;
    }
    if (s_have_recovery_sample && sample.rx_time_ms == s_last_vision_rx_ms)
    {
        return true;
    }
    if (now_ms < s_recovery_command_ready_ms)
    {
        return true;
    }
    BallActuator_GetStatus(&actuator);
    if (!actuator.actual_angle_valid)
    {
        s_status.fault_reason = 2U;
        return false;
    }
    if (!s_have_recovery_sample)
    {
        dt_s = 0.02f;
    }
    else
    {
        dt_s = (float)(sample.rx_time_ms - s_last_vision_rx_ms) / 1000.0f;
        if (dt_s < 0.01f)
            dt_s = 0.01f;
        if (dt_s > BALL_CONTROL_SAMPLE_MAX_S)
            dt_s = BALL_CONTROL_SAMPLE_MAX_S;
    }
    if (!BallStateEstimator_Update(&s_recovery_estimator, sample.position_cm, dt_s))
    {
        /* 单次视觉跳变由估计器抑制；它不算 Q4
         * 故障，先前的有效指令保持生效。 */
        return true;
    }
    {
        float distance_cm = Q4StaticRecovery_Abs(s_recovery_estimator.position_cm);
        float speed_abs = Q4StaticRecovery_Abs(s_recovery_estimator.velocity_cm_s);
        if (!s_recovery_breakaway_active && (distance_cm > BALL_OUTER_STICTION_ERROR_CM) &&
            (speed_abs <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S))
        {
            if (s_recovery_still_started_ms == 0U)
            {
                s_recovery_still_started_ms = now_ms;
            }
            else if ((now_ms - s_recovery_still_started_ms) >= BALL_CONTROL_STILL_WINDOW_MS &&
                     s_recovery_breakaway_count < BALL_CONTROL_REKICK_MAX_COUNT)
            {
                ++s_recovery_breakaway_count;
                s_recovery_breakaway_active = true;
                s_recovery_breakaway_limit_deg =
                    Q4_RECOVERY_BREAKAWAY_START_DEG +
                    ((float)s_recovery_breakaway_count * Q4_RECOVERY_BREAKAWAY_STEP_DEG);
                if (s_recovery_breakaway_limit_deg > Q4_RECOVERY_MAX_ANGLE_DEG)
                {
                    s_recovery_breakaway_limit_deg = Q4_RECOVERY_MAX_ANGLE_DEG;
                }
                s_recovery_breakaway_started_ms = now_ms;
                s_recovery_still_started_ms = 0U;
            }
        }
        else if ((distance_cm <= BALL_OUTER_STICTION_ERROR_CM) ||
                 (speed_abs > BALL_OUTER_STICTION_RELEASE_SPEED_CM_S))
        {
            s_recovery_still_started_ms = 0U;
        }
        s_status.breakaway_count = s_recovery_breakaway_count;
    }
    profile = Q4StaticRecovery_SelectProfile();
    BallOuterPd_SetProfile(&s_recovery_pd, profile);
    control_error_cm = -s_recovery_estimator.position_cm;
    if (profile == BALL_OUTER_PROFILE_RETURN_CRUISE)
    {
        /* Q3 已表征约 150 ms 的视觉反馈延迟。
         * 仅在 cruise 阶段将延迟作为 P 的提前量；制动/hold 阶段
         * 保留实测误差，使 D 在零点附近不被重复计入。 */
        float lead_time_s =
            (control_error_cm < 0.0f) ? Q4_RECOVERY_LEAD_TIME_S_ERROR_NEG : Q4_RECOVERY_LEAD_TIME_S;
        control_error_cm -= s_recovery_estimator.velocity_cm_s * lead_time_s;
    }
    s_status.control_error_cm = control_error_cm;
    {
        s_status.control_profile = (uint8_t)profile;
        if (s_recovery_breakaway_active && (profile != BALL_OUTER_PROFILE_NORMAL))
        {
            /* Q3 仅在起步时使用 8 rpm。一旦检测到运动，
             * 在下一个目标之前恢复其正常的 6-rpm profile。 */
            if (!BallActuator_PrepareAbsolutePosition(
                    Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA))
            {
                return true;
            }
            s_recovery_breakaway_active = false;
            s_recovery_command_ready_ms = now_ms + BALL_CONTROL_NORMAL_PARAM_GAP_MS;
            return true;
        }
    }
    if (s_recovery_breakaway_active && (s_recovery_breakaway_started_ms != 0U) &&
        ((now_ms - s_recovery_breakaway_started_ms) >= Q4_BREAKAWAY_HOLD_MS))
    {
        /* Q3 规模的脉冲是有限的。当机械凹坑未释放时，
         * 不要把起步角度当作持续驱动保持。 */
        s_recovery_breakaway_active = false;
        s_recovery_still_started_ms = now_ms;
        (void)BallActuator_PrepareAbsolutePosition(
            Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA);
        s_recovery_command_ready_ms = now_ms + BALL_CONTROL_NORMAL_PARAM_GAP_MS;
        return true;
    }
    if (!BallOuterPd_Update(&s_recovery_pd,
                            control_error_cm,
                            s_recovery_estimator.velocity_cm_s,
                            dt_s,
                            actuator.actual_angle_deg,
                            &output))
    {
        s_status.fault_reason = 3U;
        return false;
    }
    if (!(s_recovery_breakaway_active &&
          (s_status.control_profile == (uint8_t)BALL_OUTER_PROFILE_NORMAL)))
    {
        float fast_motor_deg;
        if (Q4StaticRecovery_ComputeFastMotor(
                control_error_cm, s_recovery_estimator.velocity_cm_s, profile, &fast_motor_deg))
        {
            output.motor_deg = fast_motor_deg;
            output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(fast_motor_deg);
        }
    }
    Q4StaticRecovery_ApplyMeasuredDistanceLimit(
        s_recovery_estimator.position_cm, profile, &output.motor_deg);
    /* Q4 恢复有自己分级限幅。只有当静止再踢之后才可
     * 超过冻结的 Q3 正常限值，并被硬性裁剪在
     * 实测的 25 度安全上限。 */
    if (output.motor_deg > Q4_RECOVERY_MAX_ANGLE_DEG)
    {
        output.motor_deg = Q4_RECOVERY_MAX_ANGLE_DEG;
        output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
    }
    else if (output.motor_deg < -Q4_RECOVERY_MAX_ANGLE_DEG)
    {
        output.motor_deg = -Q4_RECOVERY_MAX_ANGLE_DEG;
        output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
    }
    if (s_recovery_breakaway_active &&
        (s_status.control_profile == (uint8_t)BALL_OUTER_PROFILE_NORMAL))
    {
        /* 扰动后的第一个视觉采样仍可能是陈旧数据。
         * 强制首次踢击方向与给定的扰动相反，而不是
         * 相信那个旧的位置误差；新采样到达后，
         * 保持正常的按误差方向的有限再踢行为。 */
        if (!s_have_recovery_sample)
        {
            output.motor_deg = (s_status.disturbance_angle_deg >= 0.0f)
                                   ? -s_recovery_breakaway_limit_deg
                                   : s_recovery_breakaway_limit_deg;
        }
        else if (output.motor_deg < 0.0f)
        {
            output.motor_deg = -s_recovery_breakaway_limit_deg;
        }
        else if (output.motor_deg > 0.0f)
        {
            output.motor_deg = s_recovery_breakaway_limit_deg;
        }
        else
        {
            if (control_error_cm > 0.0f)
            {
                output.motor_deg = -s_recovery_breakaway_limit_deg;
            }
            else if (control_error_cm < 0.0f)
            {
                output.motor_deg = s_recovery_breakaway_limit_deg;
            }
            else
            {
                output.motor_deg = (s_status.disturbance_angle_deg >= 0.0f)
                                       ? -s_recovery_breakaway_limit_deg
                                       : s_recovery_breakaway_limit_deg;
            }
        }
        output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
    }
    if (s_direct_recovery_active)
    {
        float direct_limit_motor_deg;
        if (PendulumGeometry_BeamDegToMotorDeg(Q4_RECOVERY_DIRECT_BEAM_LIMIT_DEG,
                                               &direct_limit_motor_deg))
        {
            if (output.motor_deg > direct_limit_motor_deg)
            {
                output.motor_deg = direct_limit_motor_deg;
            }
            else if (output.motor_deg < -direct_limit_motor_deg)
            {
                output.motor_deg = -direct_limit_motor_deg;
            }
            output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
        }
    }
    if ((!s_have_recovery_sample ||
         Q4StaticRecovery_Abs(output.motor_deg - s_last_commanded_angle_deg) >=
             BALL_CONTROL_COMMAND_DELTA_DEG) &&
        ((!s_have_recovery_sample) ||
         ((now_ms - s_last_command_send_ms) >= Q4_RECOVERY_COMMAND_PERIOD_MS)))
    {
        s_last_command_send_ms = now_ms;
        if (!BallActuator_SetAbsoluteAngleDeg(output.motor_deg))
        {
            /* 临时的 X42S EE/超时将在下一个视觉采样时重试。
             * 视觉与执行器反馈看门狗，以及整体恢复超时，
             * 仍是故障边界。 */
            return true;
        }
        s_last_commanded_angle_deg = output.motor_deg;
    }
    s_last_vision_rx_ms = sample.rx_time_ms;
    s_last_valid_vision_ms = now_ms;
    s_have_recovery_sample = true;
    s_status.target_cm = 0.0f;
    s_status.measured_cm = s_recovery_estimator.position_cm;
    s_status.measured_velocity_cm_s = s_recovery_estimator.velocity_cm_s;
    s_status.error_cm = -s_recovery_estimator.position_cm;
    s_status.control_output_deg = output.motor_deg;
    s_status.recovery_limit_deg = s_recovery_breakaway_limit_deg;
    s_status.commanded_angle_deg = s_last_commanded_angle_deg;
    s_status.actual_angle_deg = actuator.actual_angle_deg;
    s_status.vision_sequence = sample.sequence;
    s_status.vision_age_ms = now_ms - sample.rx_time_ms;
    s_status.target_reached =
        (Q4StaticRecovery_Abs(s_recovery_estimator.position_cm) <= Q4_HOLD_POSITION_TOLERANCE_CM) &&
        (Q4StaticRecovery_Abs(s_recovery_estimator.velocity_cm_s) <= Q4_HOLD_SPEED_LIMIT_CM_S);
    s_status.recovery_elapsed_ms =
        (s_recovery_started_ms == 0U) ? 0U : (now_ms - s_recovery_started_ms);
    return true;
}

static void Q4StaticRecovery_Enter(Q4StaticState state, uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
    s_result_log_pending = true;
}

static float Q4StaticRecovery_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* Q4 专用快速路径。Q3 控制器仍是参考实现，
 * 但其刻意放慢的成形光束轨迹不适合 12--18 度扰动
 * 之后的情形。此路径保留实测符号
 * 与几何表，同时给 Q4 一个有界的位置/速度指令。 */
static bool Q4StaticRecovery_ComputeFastMotor(float error_cm,
                                              float velocity_cm_s,
                                              BallOuterPdProfile profile,
                                              float *motor_deg)
{
    float kp = Q4_RECOVERY_KP_BEAM_DEG_PER_CM;
    float kd = Q4_RECOVERY_KD_BEAM_DEG_PER_CM_S;
    float beam_limit = Q4_RECOVERY_BEAM_LIMIT_DEG;
    float beam_deg;

    /* 实测机构具有方向不对称性：正位置误差
     * （球位于负侧）在 Q4 日志中是较慢的恢复方向。
     * 对相反方向保留标称增益，
     * 并在此应用此 Q4 专用修正。 */
    if (error_cm > 0.0f)
    {
        kp = Q4_RECOVERY_KP_BEAM_DEG_PER_CM_ERROR_POS;
    }

    if (profile == BALL_OUTER_PROFILE_RETURN_BRAKE)
    {
        kp = Q4_RECOVERY_BRAKE_KP_BEAM_DEG_PER_CM;
        kd = Q4_RECOVERY_BRAKE_KD_BEAM_DEG_PER_CM_S;
        /* 当位置与速度具有相同物理符号时，球仍在远离零。
         * 保留一定阻尼，但减小延迟速度项，
         * 使其无法增强外行阶段。
         * 一旦速度反向，保留完整的制动阻尼。 */
        if ((error_cm * velocity_cm_s) < 0.0f)
        {
            kd = Q4_RECOVERY_BRAKE_KD_OUTWARD_BEAM_DEG_PER_CM_S;
        }
        beam_limit = Q4_RECOVERY_BRAKE_BEAM_LIMIT_DEG;
        if (error_cm < 0.0f)
        {
            beam_limit = Q4_RECOVERY_BRAKE_BEAM_LIMIT_DEG_ERROR_NEG;
        }
    }
    else if (profile == BALL_OUTER_PROFILE_RETURN_HOLD)
    {
        kp = Q4_RECOVERY_HOLD_KP_BEAM_DEG_PER_CM;
        kd = Q4_RECOVERY_HOLD_KD_BEAM_DEG_PER_CM_S;
        beam_limit = Q4_RECOVERY_HOLD_BEAM_LIMIT_DEG;
    }
    else
    {
        if (error_cm > 0.0f)
        {
            beam_limit = Q4_RECOVERY_BEAM_LIMIT_DEG_ERROR_POS;
            kd = Q4_RECOVERY_KD_BEAM_DEG_PER_CM_S_ERROR_POS;
        }
        else if (error_cm < 0.0f)
        {
            beam_limit = Q4_RECOVERY_BEAM_LIMIT_DEG_ERROR_NEG;
        }
        if ((profile == BALL_OUTER_PROFILE_RETURN_CRUISE) && ((error_cm * velocity_cm_s) < 0.0f))
        {
            kd = Q4_RECOVERY_CRUISE_KD_OUTWARD_BEAM_DEG_PER_CM_S;
        }
    }
    beam_deg = Q3_POSITION_TO_MOTOR_SIGN * (kp * error_cm - kd * velocity_cm_s);
    if (beam_deg > beam_limit)
        beam_deg = beam_limit;
    if (beam_deg < -beam_limit)
        beam_deg = -beam_limit;
    return PendulumGeometry_BeamDegToMotorDeg(beam_deg, motor_deg) &&
           (Q4StaticRecovery_Abs(*motor_deg) <= Q4_RECOVERY_MAX_ANGLE_DEG);
}

static bool Q4StaticRecovery_Ready(uint32_t now_ms)
{
    BallActuatorStatus actuator;
    VisionLinkSample sample;

    BallActuator_GetStatus(&actuator);
    if (!actuator.online || !actuator.options_valid || !actuator.firmware_x ||
        !actuator.foc_closed_loop)
    {
        return false;
    }
    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm) ||
        !isfinite(sample.velocity_cm_s) || sample.confidence_pct < BALL_CONTROL_CONFIDENCE_MIN ||
        (now_ms - sample.rx_time_ms) > Q4_VISUAL_READY_AGE_MS)
    {
        return false;
    }
    /* 每次自主试验都从真正居中静止开始。在宣告平台就绪之前，
     * 要求位置与速度限值持续满足，而非在残余恢复后的
     * 某一幸运帧上满足。 */
    if ((Q4StaticRecovery_Abs(sample.position_cm) <= Q4_START_POSITION_TOLERANCE_CM) &&
        (Q4StaticRecovery_Abs(sample.velocity_cm_s) <= Q4_HOLD_SPEED_LIMIT_CM_S))
    {
        if (s_ready_center_since_ms == 0U)
        {
            s_ready_center_since_ms = now_ms;
        }
        return (now_ms - s_ready_center_since_ms) >= Q4_READY_CONFIRM_MS;
    }
    s_ready_center_since_ms = 0U;
    return false;
}

static bool Q4StaticRecovery_StartDirectRecovery(uint32_t now_ms)
{
    s_direct_recovery_active = true;
    s_recovery_breakaway_active = true;
    s_recovery_breakaway_limit_deg = Q4_RECOVERY_DIRECT_BREAKAWAY_DEG;
    s_recovery_breakaway_count = 0U;
    s_recovery_still_started_ms = 0U;
    s_recovery_breakaway_started_ms = now_ms;
    s_status.breakaway_count = 0U;
    s_status.acceptance_passed = false;
    s_status.target_reached = false;
    s_recovery_started_ms = now_ms;
    if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
        !BallActuator_PrepareAbsolutePosition(
            Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, BALL_CONTROL_BREAKAWAY_SPEED_RPM, Q4_MAX_CURRENT_MA))
    {
        return false;
    }
    BallOuterPd_Init(&s_recovery_pd);
    BallStateEstimator_Init(&s_recovery_estimator);
    s_have_recovery_sample = false;
    s_hold_started_ms = 0U;
    s_hold_candidate_ms = 0U;
    s_hold_candidate_ms = 0U;
    s_last_commanded_angle_deg = 0.0f;
    s_recovery_command_ready_ms = now_ms + Q4_RECOVERY_START_DELAY_MS;
    s_last_command_send_ms = 0U;
    Q4StaticRecovery_Enter(Q4_STATIC_RECOVER, now_ms);
    return true;
}

static bool Q4StaticRecovery_SafeVision(void)
{
    VisionLinkSample sample;
    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm))
    {
        return true;
    }
    (void)sample;
    return true;
}

void Q4StaticRecovery_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = Q4_STATIC_WAIT;
    s_random_state = 0x5A3CU;
    s_disturb_stage = 0U;
    s_disturb_started_ms = 0U;
    s_zero_stage = 0U;
    s_zero_started_ms = 0U;
    s_zero_last_command_ms = 0U;
    s_zero_nudge_sent = false;
    s_result_log_pending = false;
    BallOuterPd_Init(&s_recovery_pd);
    s_last_vision_rx_ms = 0U;
    s_last_valid_vision_ms = 0U;
    s_have_recovery_sample = false;
    s_hold_started_ms = 0U;
    s_last_commanded_angle_deg = 0.0f;
    s_recovery_command_ready_ms = 0U;
    s_last_command_send_ms = 0U;
    s_recovery_breakaway_active = false;
    s_direct_recovery_active = false;
    s_startup_zero_pending = true;
    s_recovery_breakaway_count = 0U;
    s_recovery_still_started_ms = 0U;
    s_recovery_breakaway_started_ms = 0U;
    s_recovery_started_ms = 0U;
    s_ready_center_since_ms = 0U;
    s_fault_zero_pending = false;
    s_recovery_breakaway_limit_deg = Q4_RECOVERY_BREAKAWAY_START_DEG;
    s_status.breakaway_count = 0U;
    s_status.acceptance_passed = false;
}

void Q4StaticRecovery_Task(uint32_t now_ms)
{
    if ((s_status.state == Q4_STATIC_FINISH) || (s_status.state == Q4_STATIC_FAULT))
    {
        return;
    }

    if (!Q4StaticRecovery_SafeVision())
    {
        Q4StaticRecovery_Fault(now_ms);
        return;
    }

    switch (s_status.state)
    {
    case Q4_STATIC_WAIT:
    {
        BallActuatorStatus actuator;
        VisionLinkSample sample;
        BallActuator_GetStatus(&actuator);
        /* 启动时执行器可能仍报告上一会话的
                 * 角度，而视觉采样已偏离中心。先将电机会话
                 * 归零；否则直接恢复可能继承陈旧的
                 * 光束角度并产生无效的 run=0。 */
        if (s_startup_zero_pending)
        {
            if (actuator.actual_angle_valid)
            {
                if (!Q4StaticRecovery_BeginZero(now_ms))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
            }
            break;
        }
        if (actuator.actual_angle_valid &&
            Q4StaticRecovery_Abs(actuator.actual_angle_deg) > Q4_ZERO_RETURN_TOLERANCE_DEG)
        {
            if (!Q4StaticRecovery_BeginZero(now_ms))
            {
                Q4StaticRecovery_Fault(now_ms);
            }
            break;
        }
        if (VisionLink_GetLatest(&sample) && sample.valid && isfinite(sample.position_cm) &&
            sample.confidence_pct >= Q4_RECOVERY_CONFIDENCE_MIN &&
            (now_ms - sample.rx_time_ms) <= Q4_VISUAL_READY_AGE_MS &&
            Q4StaticRecovery_Abs(sample.position_cm) > 2.0f)
        {
            BallActuator_SetCommandSession(true);
            if (!Q4StaticRecovery_StartDirectRecovery(now_ms))
            {
                Q4StaticRecovery_Fault(now_ms);
            }
            break;
        }
    }
        if (((now_ms - s_status.state_started_ms) >= Q4_NEXT_RUN_GAP_MS) &&
            Q4StaticRecovery_Ready(now_ms))
        {
            /* 类 LFSR 的确定性序列：只有方向是伪随机的。
                 * 幅度受实测的 +/-18 度安全测试限值约束。 */
            s_random_state = (uint16_t)(s_random_state * 25173U + 13849U);
            {
                uint16_t span10 =
                    (uint16_t)((Q4_DISTURBANCE_MAX_ANGLE_DEG - Q4_DISTURBANCE_MIN_ANGLE_DEG) *
                                   10.0f +
                               0.5f);
                uint16_t magnitude10 = (uint16_t)(Q4_DISTURBANCE_MIN_ANGLE_DEG * 10.0f + 0.5f);
                if (span10 > 0U)
                {
                    magnitude10 = (uint16_t)(magnitude10 + (s_random_state % (span10 + 1U)));
                }
                /* 正电机角度把球驱向视觉
                     * -5 cm；相反符号测试另一侧。 */
                s_status.disturbance_angle_deg = ((s_random_state & 1U) != 0U)
                                                     ? (float)magnitude10 / 10.0f
                                                     : -(float)magnitude10 / 10.0f;
            }
            BallActuator_SetCommandSession(true);
            /* X42S 在 enable、F1 参数与
                 * CD 位置之间需要指令间隔。若在一次 UART 突发中发送全部三条，
                 * 电机可能停留在先前的角度，而状态机
                 * 却误以为扰动已开始。 */
            if (!BallActuator_ClearProtection() || !BallActuator_Enable(true))
            {
                Q4StaticRecovery_Fault(now_ms);
            }
            else
            {
                ++s_status.run_sequence;
                s_ready_center_since_ms = 0U;
                s_disturb_stage = 1U;
                s_disturb_started_ms = 0U;
                Q4StaticRecovery_Enter(Q4_STATIC_DISTURB, now_ms);
            }
        }
        break;

    case Q4_STATIC_DISTURB:
        if (s_disturb_stage == 1U)
        {
            if ((now_ms - s_status.state_started_ms) >= 100U)
            {
                if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
                    !BallActuator_PrepareAbsolutePosition(
                        Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_disturb_stage = 2U;
                    s_status.state_started_ms = now_ms;
                }
            }
        }
        else if (s_disturb_stage == 2U)
        {
            if ((now_ms - s_status.state_started_ms) >= 20U)
            {
                if (!BallActuator_SetAbsoluteAngleDeg(s_status.disturbance_angle_deg))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_disturb_stage = 3U;
                    s_disturb_started_ms = now_ms;
                }
            }
        }
        else if ((s_disturb_stage == 3U) &&
                 ((now_ms - s_disturb_started_ms) >= Q4_DISTURBANCE_DURATION_MS))
        {
            BallActuatorStatus actuator;
            BallActuator_GetStatus(&actuator);
            if (actuator.actual_angle_valid &&
                (Q4StaticRecovery_Abs(actuator.actual_angle_deg - s_status.disturbance_angle_deg) <=
                 1.0f))
            {
                if (!BallActuator_PrepareAbsolutePosition(Q4_ACCEL_RPM_S,
                                                          Q4_DECEL_RPM_S,
                                                          BALL_CONTROL_BREAKAWAY_SPEED_RPM,
                                                          Q4_MAX_CURRENT_MA))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    BallOuterPd_Init(&s_recovery_pd);
                    BallStateEstimator_Init(&s_recovery_estimator);
                    s_direct_recovery_active = false;
                    s_recovery_breakaway_active = true;
                    s_recovery_breakaway_limit_deg = Q4_RECOVERY_BREAKAWAY_START_DEG;
                    s_recovery_breakaway_count = 0U;
                    s_recovery_still_started_ms = 0U;
                    s_recovery_breakaway_started_ms = now_ms;
                    s_status.breakaway_count = 0U;
                    s_status.acceptance_passed = false;
                    s_status.target_reached = false;
                    s_recovery_started_ms = now_ms;
                    s_have_recovery_sample = false;
                    s_last_valid_vision_ms = now_ms;
                    s_hold_started_ms = 0U;
                    s_hold_candidate_ms = 0U;
                    s_last_commanded_angle_deg = 0.0f;
                    s_recovery_command_ready_ms = now_ms + Q4_RECOVERY_START_DELAY_MS;
                    s_last_command_send_ms = 0U;
                    Q4StaticRecovery_Enter(Q4_STATIC_RECOVER, now_ms);
                }
            }
            else if ((now_ms - s_disturb_started_ms) >= Q4_DISTURBANCE_VERIFY_TIMEOUT_MS)
            {
                s_status.fault_reason = 7U;
                Q4StaticRecovery_Fault(now_ms);
            }
        }
        break;

    case Q4_STATIC_RECOVER:
        if (!Q4StaticRecovery_UpdateControl(now_ms))
        {
            Q4StaticRecovery_Fault(now_ms);
        }
        else if (s_status.target_reached)
        {
            if (s_hold_candidate_ms == 0U)
            {
                s_hold_candidate_ms = now_ms;
            }
            else if ((now_ms - s_hold_candidate_ms) >= Q4_HOLD_CONFIRM_MS)
            {
                s_status.acceptance_passed =
                    (s_recovery_started_ms != 0U) &&
                    ((now_ms - s_recovery_started_ms) <= Q4_RECOVERY_ACCEPT_MS);
                s_hold_started_ms = now_ms;
                Q4StaticRecovery_Enter(Q4_STATIC_HOLD, now_ms);
            }
        }
        else
        {
            s_hold_candidate_ms = 0U;
            if ((now_ms - s_status.state_started_ms) >= Q4_RECOVERY_TIMEOUT_MS)
            {
                s_status.fault_reason = 4U;
                Q4StaticRecovery_Fault(now_ms);
            }
        }
        break;

    case Q4_STATIC_HOLD:
        if (!Q4StaticRecovery_UpdateControl(now_ms))
        {
            Q4StaticRecovery_Fault(now_ms);
        }
        else
        {
            if (s_status.target_reached)
            {
                if ((now_ms - s_hold_started_ms) >= Q4_HOLD_BEFORE_ZERO_MS)
                {
                    if (!Q4StaticRecovery_BeginZero(now_ms))
                    {
                        Q4StaticRecovery_Fault(now_ms);
                    }
                    else
                    {
                        /* BeginZero 负责分级的归零指令。 */
                    }
                }
            }
            else
            {
                s_hold_candidate_ms = 0U;
                Q4StaticRecovery_Enter(Q4_STATIC_RECOVER, now_ms);
            }
        }
        break;

    case Q4_STATIC_RETURN_ZERO:
        if (s_zero_stage == 1U)
        {
            if ((now_ms - s_status.state_started_ms) >= 100U)
            {
                if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
                    !BallActuator_PrepareAbsolutePosition(
                        Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_zero_stage = 2U;
                    s_status.state_started_ms = now_ms;
                }
            }
        }
        else if (s_zero_stage == 2U)
        {
            if ((now_ms - s_status.state_started_ms) >= 20U)
            {
                if (!BallActuator_ReturnAbsoluteZero())
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_zero_stage = 3U;
                    s_zero_started_ms = now_ms;
                    s_zero_last_command_ms = now_ms;
                    s_status.commanded_angle_deg = 0.0f;
                }
            }
        }
        else if (s_zero_stage == 3U)
        {
            BallActuatorStatus actuator;
            BallActuator_GetStatus(&actuator);
            if (!s_zero_nudge_sent && ((now_ms - s_zero_started_ms) >= 500U) &&
                actuator.actual_angle_valid &&
                (Q4StaticRecovery_Abs(actuator.actual_angle_deg) > Q4_ZERO_RETURN_TOLERANCE_DEG))
            {
                /* 一个小的反向侧目标可越过 X42S 静态
                     * 位置死区；随后的 9A/04 指令便
                     * 稳定在存储的机械零点。 */
                float nudge =
                    (actuator.actual_angle_deg > 0.0f) ? -Q4_ZERO_NUDGE_DEG : Q4_ZERO_NUDGE_DEG;
                if (BallActuator_SetAbsoluteAngleDeg(nudge))
                {
                    s_zero_nudge_sent = true;
                    s_zero_last_command_ms = now_ms;
                }
            }
            else if ((now_ms - s_zero_last_command_ms) >= 250U)
            {
                /* CD 是幂等的。在等待反馈期间重试会话零目标，
                     * 使丢失一帧 UART 不会把正常归零
                     * 变成误报的 Q4 故障。 */
                if (BallActuator_ReturnAbsoluteZero())
                {
                    s_zero_last_command_ms = now_ms;
                    s_status.commanded_angle_deg = 0.0f;
                }
            }
            if (actuator.actual_angle_valid &&
                Q4StaticRecovery_Abs(actuator.actual_angle_deg) <= Q4_ZERO_RETURN_TOLERANCE_DEG)
            {
                BallActuator_SetCommandSession(false);
                s_startup_zero_pending = false;
                if (s_fault_zero_pending)
                {
                    s_fault_zero_pending = false;
                    Q4StaticRecovery_Enter(Q4_STATIC_FAULT, now_ms);
                }
                else
                {
                    Q4StaticRecovery_Enter(Q4_STATIC_WAIT, now_ms);
                }
            }
            else if ((now_ms - s_zero_started_ms) >= Q4_ZERO_RETURN_TIMEOUT_MS)
            {
                s_status.fault_reason = 5U;
                (void)BallActuator_Stop();
                (void)BallActuator_Enable(false);
                BallActuator_SetCommandSession(false);
                s_fault_zero_pending = false;
                Q4StaticRecovery_Fault(now_ms);
            }
        }
        break;

    default:
        break;
    }
}

void Q4StaticRecovery_Stop(uint32_t now_ms)
{
    (void)BallActuator_Stop();
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    Q4StaticRecovery_Enter(Q4_STATIC_FINISH, now_ms);
}

bool Q4StaticRecovery_IsActive(void)
{
    return (s_status.state != Q4_STATIC_FINISH) && (s_status.state != Q4_STATIC_FAULT);
}

bool Q4StaticRecovery_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void Q4StaticRecovery_GetStatus(Q4StaticStatus *status)
{
    if (status != NULL)
        *status = s_status;
}

const char *Q4StaticRecovery_StateName(Q4StaticState state)
{
    switch (state)
    {
    case Q4_STATIC_WAIT:
        return "WAIT";
    case Q4_STATIC_DISTURB:
        return "DISTURB";
    case Q4_STATIC_RECOVER:
        return "RECOVER";
    case Q4_STATIC_HOLD:
        return "HOLD0";
    case Q4_STATIC_RETURN_ZERO:
        return "ZERO";
    case Q4_STATIC_FINISH:
        return "FINISH";
    case Q4_STATIC_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

#endif /* Q4_STATIC_RECOVERY_TEST_ENABLED */

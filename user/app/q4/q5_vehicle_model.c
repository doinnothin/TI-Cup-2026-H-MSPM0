#include "q5_vehicle_model.h"

#include <math.h>
#include <string.h>
#include "q45_vehicle_config.h"

static float Clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float SmoothStep(float value)
{
    float t = Clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static float ArcBlend(float preview_cm, float start_cm, float end_cm, float blend_width_cm)
{
    float width = blend_width_cm;
    return SmoothStep((preview_cm - start_cm + 0.5f * width) / width) *
           SmoothStep((end_cm - preview_cm + 0.5f * width) / width);
}

float Q5VehicleModel_TurnBlend(float distance_cm, float speed_mm_s)
{
    /* 预览量跟随实际速度：减速时不应保留固定的
     * 0.4 s 提前倾角。半径/方向仍需赛道标定。 */
    float preview_cm = distance_cm + Clamp(speed_mm_s, 0.0f, Q45_Q5_CRUISE_SPEED_MM_S) * 0.1f *
                                         Q45_Q5_TURN_PREVIEW_S;
    /* 导轨固定在车身坐标系中。因此两个 U 形弯会沿相反的
     * 导轨方向推动球；若把它们当作一个无符号的
     * v^2/R 扰动，会在全局符号翻转后使第一个弯失败。
     * 在遥测中保留此带符号值，使 ay100 直接显示
     * 模型正在补偿哪一个弯。 */
    return Clamp(Q45_Q5_FIRST_TURN_ACCEL_SIGN * ArcBlend(preview_cm,
                                                         Q45_Q5_FIRST_TURN_START_CM,
                                                         Q45_Q5_FIRST_TURN_END_CM,
                                                         Q45_Q5_TURN_BLEND_DISTANCE_CM) +
                     Q45_Q5_SECOND_TURN_ACCEL_SIGN * ArcBlend(preview_cm,
                                                              Q45_Q5_SECOND_TURN_START_CM,
                                                              Q45_Q5_SECOND_TURN_END_CM,
                                                              Q45_Q5_SECOND_TURN_BLEND_DISTANCE_CM),
                 -1.0f,
                 1.0f);
}

float Q5VehicleModel_TraceSpeed(uint32_t elapsed_ms)
{
    /* 刻意与球位置和相机噪声无关。 */
    if (elapsed_ms >= Q45_Q5_ACCEL_TIME_MS)
        return Q45_Q5_CRUISE_SPEED_MM_S;
    return Q45_Q5_CRUISE_SPEED_MM_S * (float)elapsed_ms / (float)Q45_Q5_ACCEL_TIME_MS;
}

void Q5VehicleModel_Init(Q5VehicleModel *model)
{
    if (model != NULL)
        memset(model, 0, sizeof(*model));
}

static bool Q5VehicleModel_UpdateInternal(Q5VehicleModel *model,
                                          uint32_t now_ms,
                                          float left_mm_s,
                                          float right_mm_s,
                                          float distance_cm,
                                          bool final_q5_profile)
{
    uint32_t dt_ms;
    float raw_speed;
    float previous_speed;
    float raw_accel;
    float v_m_s;
    float lateral_gain;
    float longitudinal_gain;
    float turn_start_cm;
    float turn_progress_cm;
    float steady_lateral_gain;
    float entry_lateral_gain;
    float apex_extra_gain;
    float apex_ramp_start_cm;
    float apex_ramp_full_cm;
    float apex_fade_start_cm;
    float apex_fade_end_cm;
    float entry_hold_cm;
    float entry_weight;
    float apex_weight;
    if ((model == NULL) || !isfinite(left_mm_s) || !isfinite(right_mm_s) || !isfinite(distance_cm))
        return false;
    dt_ms = now_ms - model->sampled_ms;
    if (model->initialized && dt_ms < Q45_Q5_CHASSIS_SAMPLE_MS)
        return false;
    raw_speed = 0.5f * (fabsf(left_mm_s) + fabsf(right_mm_s));
    if (!model->initialized || dt_ms > Q45_VISION_TIMEOUT_MS)
    {
        model->speed_mm_s = raw_speed;
        model->longitudinal_accel_m_s2 = 0.0f;
        model->initialized = true;
    }
    else
    {
        previous_speed = model->speed_mm_s;
        model->speed_mm_s += Q45_Q5_SPEED_FILTER_ALPHA * (raw_speed - model->speed_mm_s);
        /* (mm/s)/ms 在数值上即 m/s^2；使用实际经过时间，
         * 而非当前台任务被延迟时虚构的 20 ms。 */
        raw_accel = (model->speed_mm_s - previous_speed) / (float)dt_ms;
        raw_accel =
            Clamp(raw_accel, -Q45_Q5_PARALLEL_ACCEL_LIMIT_M_S2, Q45_Q5_PARALLEL_ACCEL_LIMIT_M_S2);
        model->longitudinal_accel_m_s2 +=
            Q45_Q5_ACCEL_FILTER_ALPHA * (raw_accel - model->longitudinal_accel_m_s2);
    }
    model->sampled_ms = now_ms;
    model->turn_blend = Q5VehicleModel_TurnBlend(distance_cm, model->speed_mm_s);
    v_m_s = model->speed_mm_s * 0.001f;
    model->lateral_accel_m_s2 = model->turn_blend * v_m_s * v_m_s / Q45_Q5_TURN_RADIUS_M;
    if (!final_q5_profile)
    {
        /* 对 finalized Q5 之外的调用方保留原有规律。 */
        lateral_gain = model->turn_blend > 0.0f ? Q45_Q5_SECOND_TURN_LATERAL_FF_GAIN
                                                : Q45_Q5_FIRST_TURN_LATERAL_FF_GAIN;
    }
    else
    {
        if (model->turn_blend > 0.0f)
        {
            turn_start_cm = Q45_Q5_SECOND_TURN_START_CM;
            steady_lateral_gain = Q45_Q5_SECOND_TURN_LATERAL_FF_GAIN;
            entry_lateral_gain = Q45_Q5_SECOND_TURN_ENTRY_LATERAL_FF_GAIN;
            apex_extra_gain = Q45_Q5_SECOND_TURN_APEX_EXTRA_GAIN;
            apex_ramp_start_cm = Q45_Q5_SECOND_TURN_APEX_RAMP_START_CM;
            apex_ramp_full_cm = Q45_Q5_SECOND_TURN_APEX_RAMP_FULL_CM;
            apex_fade_start_cm = Q45_Q5_SECOND_TURN_APEX_FADE_START_CM;
            apex_fade_end_cm = Q45_Q5_SECOND_TURN_APEX_FADE_END_CM;
            entry_hold_cm = Q45_Q5_SECOND_TURN_ENTRY_GAIN_HOLD_CM;
        }
        else
        {
            turn_start_cm = Q45_Q5_FIRST_TURN_START_CM;
            steady_lateral_gain = Q45_Q5_FIRST_TURN_LATERAL_FF_GAIN;
            entry_lateral_gain = Q45_Q5_FIRST_TURN_ENTRY_LATERAL_FF_GAIN;
            apex_extra_gain = Q45_Q5_FIRST_TURN_APEX_EXTRA_GAIN;
            apex_ramp_start_cm = Q45_Q5_FIRST_TURN_APEX_RAMP_START_CM;
            apex_ramp_full_cm = Q45_Q5_FIRST_TURN_APEX_RAMP_FULL_CM;
            apex_fade_start_cm = Q45_Q5_FIRST_TURN_APEX_FADE_START_CM;
            apex_fade_end_cm = Q45_Q5_FIRST_TURN_APEX_FADE_END_CM;
            entry_hold_cm = Q45_Q5_FIRST_TURN_ENTRY_GAIN_HOLD_CM;
        }
        turn_progress_cm = distance_cm - turn_start_cm;
        entry_weight =
            1.0f - SmoothStep((turn_progress_cm - entry_hold_cm) / Q45_Q5_TURN_ENTRY_GAIN_FADE_CM);
        apex_weight = SmoothStep((turn_progress_cm - apex_ramp_start_cm) /
                                 (apex_ramp_full_cm - apex_ramp_start_cm)) *
                      (1.0f - SmoothStep((turn_progress_cm - apex_fade_start_cm) /
                                         (apex_fade_end_cm - apex_fade_start_cm)));
        lateral_gain = steady_lateral_gain +
                       entry_weight * (entry_lateral_gain - steady_lateral_gain) +
                       apex_weight * apex_extra_gain;
        if (model->turn_blend > 0.0f)
        {
            lateral_gain *= Q45_Q5_SECOND_TURN_FF_POLARITY;
        }
    }
    /* 不要在启动瞬间注入实测的底盘加速度：初始轮速
     * 瞬态会在静态环建立出发平衡之前把球向后推。
     * 在第一个弯之前该增益恰好为 1.0，因此巡航与两个弯的
     * 设置都保持不变。 */
    longitudinal_gain = Q45_Q5_LONGITUDINAL_FF_GAIN *
                        SmoothStep(distance_cm / Q45_Q5_LONGITUDINAL_FF_RAMP_DISTANCE_CM);
    model->parallel_accel_m_s2 = Clamp(longitudinal_gain * model->longitudinal_accel_m_s2 +
                                           lateral_gain * model->lateral_accel_m_s2,
                                       -Q45_Q5_PARALLEL_ACCEL_LIMIT_M_S2,
                                       Q45_Q5_PARALLEL_ACCEL_LIMIT_M_S2);
    return true;
}

bool Q5VehicleModel_Update(
    Q5VehicleModel *model, uint32_t now_ms, float left_mm_s, float right_mm_s, float distance_cm)
{
    return Q5VehicleModel_UpdateInternal(model, now_ms, left_mm_s, right_mm_s, distance_cm, false);
}

bool Q5VehicleModel_UpdateFinalQ5(
    Q5VehicleModel *model, uint32_t now_ms, float left_mm_s, float right_mm_s, float distance_cm)
{
    return Q5VehicleModel_UpdateInternal(model, now_ms, left_mm_s, right_mm_s, distance_cm, true);
}

#include "q5_dynamic_ball_control.h"

#include <math.h>
#include <string.h>
#include "app/q3/pendulum_geometry.h"
#include "app/q3/q3_control_config.h"
#include "q45_vehicle_config.h"

#define Q5_RAD_TO_DEG 57.2957795131f

static float Clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float SmoothStep(float value)
{
    float t = Clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

void Q5DynamicBallControl_Init(Q5DynamicBallController *controller)
{
    if (controller != NULL)
        memset(controller, 0, sizeof(*controller));
}

bool Q5DynamicBallControl_Update(Q5DynamicBallController *controller,
                                 const Q5DynamicBallInput *input,
                                 Q5DynamicBallOutput *output)
{
    float actual_beam, distance, speed, hold_weight, brake_weight = 0.0f;
    float stop_distance, kp, kd, integral_old, target, remaining;
    float requested_rate, max_rate_step, step, previous_angle, bounded_angle;
    if ((controller == NULL) || (input == NULL) || (output == NULL) ||
        !isfinite(input->measured_position_cm) || !isfinite(input->error_cm) ||
        !isfinite(input->velocity_cm_s) || !isfinite(input->parallel_accel_m_s2) ||
        !isfinite(input->actual_motor_deg) || !isfinite(input->dt_s) || (input->dt_s <= 0.0f) ||
        (input->dt_s > 0.25f) || (fabsf(input->actual_motor_deg) > Q3_MOTOR_SAFE_LIMIT_DEG))
        return false;

    memset(output, 0, sizeof(*output));
    actual_beam = PendulumGeometry_MotorDegToBeamDeg(input->actual_motor_deg);
    if (!controller->initialized)
    {
        controller->beam_angle_deg =
            Clamp(actual_beam, -Q45_Q5_TOTAL_BEAM_LIMIT_DEG, Q45_Q5_TOTAL_BEAM_LIMIT_DEG);
        controller->beam_rate_deg_s = 0.0f;
        controller->integral_beam_deg = 0.0f;
        controller->initialized = true;
    }

    /* 复用实测的 Q4 增益；对阶段做混合，使越过阈值时
     * 不会突然切换所要求的倾角。+/-1 cm 永不作为 hold 开关。
     * Q4 的 5/7 cm 制动回退属于大幅静态抛掷，不适用于 Q5。 */
    distance = fabsf(input->measured_position_cm);
    speed = fabsf(input->velocity_cm_s);
    hold_weight =
        (1.0f - SmoothStep((distance - Q45_Q5_INTERNAL_POSITION_CM) /
                           (Q45_Q5_HOLD_BLEND_END_CM - Q45_Q5_INTERNAL_POSITION_CM))) *
        (1.0f - SmoothStep((speed - Q45_Q5_INTERNAL_SPEED_CM_S) /
                           (Q45_Q5_HOLD_BLEND_END_SPEED_CM_S - Q45_Q5_INTERNAL_SPEED_CM_S)));
    if (input->measured_position_cm * input->velocity_cm_s < 0.0f)
    {
        stop_distance = speed * (Q3_RETURN_FEEDBACK_DELAY_MS * 0.001f) +
                        speed * speed / (2.0f * Q3_RETURN_DECELERATION_CM_S2) +
                        Q45_Q5_BRAKE_MARGIN_CM;
        brake_weight = SmoothStep((stop_distance - distance + 0.5f * Q45_Q5_BRAKE_BLEND_CM) /
                                  Q45_Q5_BRAKE_BLEND_CM);
    }
    kp = input->error_cm > 0.0f ? Q45_Q5_KP_ERROR_POS : Q45_Q5_KP_BEAM_DEG_PER_CM;
    kd = Q45_Q5_KD_BEAM_DEG_PER_CM_S;
    kp += brake_weight * (Q45_Q5_BRAKE_KP - kp);
    kd += brake_weight * (Q45_Q5_BRAKE_KD - kd);
    kp += hold_weight * (Q45_Q5_HOLD_KP - kp);
    kd += hold_weight * (Q45_Q5_HOLD_KD - kd);
    output->profile = hold_weight >= 0.5f ? 0U : (brake_weight >= 0.5f ? 2U : 1U);
    output->p_beam_deg = Q3_POSITION_TO_MOTOR_SIGN * kp * input->error_cm;
    output->d_beam_deg = Clamp(-Q3_POSITION_TO_MOTOR_SIGN * kd * input->velocity_cm_s,
                               -Q45_Q5_D_TERM_LIMIT_DEG,
                               Q45_Q5_D_TERM_LIMIT_DEG);

    /* 零相对加速度平衡：tan(theta)=a_parallel/g。
     * 5/7 影响滚动动力学，但在此平衡中相互抵消。 */
    output->feedforward_beam_deg =
        Clamp(Q45_Q5_FEEDFORWARD_BEAM_SIGN * Q45_Q5_FEEDFORWARD_GAIN *
                  atanf(input->parallel_accel_m_s2 / Q45_Q5_GRAVITY_M_S2) * Q5_RAD_TO_DEG,
              -Q45_Q5_FEEDFORWARD_LIMIT_DEG,
              Q45_Q5_FEEDFORWARD_LIMIT_DEG);

    integral_old = controller->integral_beam_deg *
                   Clamp(1.0f - Q45_Q5_INTEGRAL_LEAK_PER_S * input->dt_s, 0.0f, 1.0f);
    controller->integral_beam_deg = integral_old;
    if ((distance <= Q45_Q5_INTEGRAL_ACTIVE_POSITION_CM) &&
        (speed <= Q45_Q5_INTEGRAL_ACTIVE_SPEED_CM_S) &&
        (fabsf(input->parallel_accel_m_s2) <= Q45_Q5_INTEGRAL_ACTIVE_ACCEL_M_S2))
    {
        controller->integral_beam_deg =
            Clamp(integral_old + Q3_POSITION_TO_MOTOR_SIGN * Q45_Q5_KI_BEAM_DEG_PER_CM_S *
                                     input->error_cm * input->dt_s,
                  -Q45_Q5_INTEGRAL_LIMIT_DEG,
                  Q45_Q5_INTEGRAL_LIMIT_DEG);
    }
    output->raw_beam_deg = output->p_beam_deg + output->d_beam_deg + output->feedforward_beam_deg +
                           controller->integral_beam_deg;
    if ((fabsf(output->raw_beam_deg) > Q45_Q5_TOTAL_BEAM_LIMIT_DEG) &&
        ((controller->integral_beam_deg - integral_old) * output->raw_beam_deg > 0.0f))
    {
        output->raw_beam_deg -= controller->integral_beam_deg - integral_old;
        controller->integral_beam_deg = integral_old;
    }
    output->i_beam_deg = controller->integral_beam_deg;
    output->limited_beam_deg =
        Clamp(output->raw_beam_deg, -Q45_Q5_TOTAL_BEAM_LIMIT_DEG, Q45_Q5_TOTAL_BEAM_LIMIT_DEG);
    target = Clamp(output->limited_beam_deg,
                   actual_beam - Q45_Q5_COMMAND_LEAD_LIMIT_DEG,
                   actual_beam + Q45_Q5_COMMAND_LEAD_LIMIT_DEG);
    target = Clamp(target, -Q45_Q5_TOTAL_BEAM_LIMIT_DEG, Q45_Q5_TOTAL_BEAM_LIMIT_DEG);

    previous_angle = controller->beam_angle_deg;
    remaining = target - previous_angle;
    /* 在指令轨迹到达目标之前使其减速。
     * 对最终的成形角度以及原始 PID 求和都做硬限幅。 */
    requested_rate = sqrtf(2.0f * Q45_Q5_BEAM_ACCEL_LIMIT_DEG_S2 * fabsf(remaining));
    requested_rate = Clamp(requested_rate, 0.0f, Q45_Q5_BEAM_RATE_LIMIT_DEG_S);
    if (remaining < 0.0f)
        requested_rate = -requested_rate;
    max_rate_step = Q45_Q5_BEAM_ACCEL_LIMIT_DEG_S2 * input->dt_s;
    controller->beam_rate_deg_s +=
        Clamp(requested_rate - controller->beam_rate_deg_s, -max_rate_step, max_rate_step);
    step = controller->beam_rate_deg_s * input->dt_s;
    if ((step * remaining >= 0.0f) && (fabsf(step) >= fabsf(remaining)))
    {
        step = remaining;
        controller->beam_rate_deg_s = 0.0f;
    }
    bounded_angle =
        Clamp(previous_angle + step, -Q45_Q5_TOTAL_BEAM_LIMIT_DEG, Q45_Q5_TOTAL_BEAM_LIMIT_DEG);
    bounded_angle = Clamp(bounded_angle,
                          actual_beam - Q45_Q5_COMMAND_LEAD_LIMIT_DEG,
                          actual_beam + Q45_Q5_COMMAND_LEAD_LIMIT_DEG);
    bounded_angle = Clamp(bounded_angle, -Q45_Q5_TOTAL_BEAM_LIMIT_DEG, Q45_Q5_TOTAL_BEAM_LIMIT_DEG);
    if (fabsf(bounded_angle - (previous_angle + step)) > 0.0001f)
        controller->beam_rate_deg_s = 0.0f;
    controller->beam_angle_deg = bounded_angle;
    output->shaped_beam_deg = bounded_angle;
    output->limited = fabsf(output->raw_beam_deg - output->limited_beam_deg) > 0.0001f ||
                      fabsf(target - output->limited_beam_deg) > 0.0001f;
    output->predicted_position_cm =
        input->measured_position_cm + input->velocity_cm_s * Q45_Q5_PREDICTION_TIME_S;
    return PendulumGeometry_BeamDegToMotorDeg(output->shaped_beam_deg, &output->motor_deg);
}

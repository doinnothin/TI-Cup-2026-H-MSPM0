#include "pendulum_geometry.h"

#include <math.h>
#include <stddef.h>

#include "app/q3/q3_control_config.h"

typedef struct
{
    float motor_deg;
    float end_height_mm;
} PendulumGeometryPoint;

/* 用户以 2 度为增量测得的右端累积上升量：
 * 每步增量依次为 1,1,1,1,1,1,2,2,2 mm。该表有意
 * 存储原始的分段结果，而不是假装它是单一精确的
 * 传动比。 */
static const PendulumGeometryPoint s_points[] = {{0.0f, 0.0f},
                                                 {2.0f, 1.0f},
                                                 {4.0f, 2.0f},
                                                 {6.0f, 3.0f},
                                                 {8.0f, 4.0f},
                                                 {10.0f, 5.0f},
                                                 {12.0f, 6.0f},
                                                 {14.0f, 8.0f},
                                                 {16.0f, 10.0f},
                                                 {18.0f, 12.0f}};

#define Q3_GEOMETRY_POINT_COUNT (sizeof(s_points) / sizeof(s_points[0]))
#define Q3_RAD_TO_DEG 57.2957795131f
#define Q3_DEG_TO_RAD 0.01745329252f

static float PendulumGeometry_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PendulumGeometry_Sign(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float PendulumGeometry_Interpolate(float x, float x0, float y0, float x1, float y1)
{
    if (x1 == x0)
    {
        return y0;
    }
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

bool PendulumGeometry_EndHeightMmToMotorDeg(float height_mm, float *motor_deg)
{
    size_t i;
    float magnitude = fabsf(height_mm);
    if (!motor_deg || !isfinite(height_mm))
        return false;
    for (i = 1; i < Q3_GEOMETRY_POINT_COUNT; ++i)
    {
        if (magnitude <= s_points[i].end_height_mm)
        {
            *motor_deg = PendulumGeometry_Sign(height_mm) *
                         PendulumGeometry_Interpolate(magnitude,
                                                      s_points[i - 1].end_height_mm,
                                                      s_points[i - 1].motor_deg,
                                                      s_points[i].end_height_mm,
                                                      s_points[i].motor_deg);
            return true;
        }
    }
    return false;
}

float PendulumGeometry_MotorDegToEndHeightMm(float motor_angle_deg)
{
    float magnitude = PendulumGeometry_Abs(motor_angle_deg);
    float height_mm;
    size_t index;

    if (magnitude > Q3_MOTOR_SAFE_LIMIT_DEG)
    {
        magnitude = Q3_MOTOR_SAFE_LIMIT_DEG;
    }

    for (index = 1U; index < Q3_GEOMETRY_POINT_COUNT; ++index)
    {
        if (magnitude <= s_points[index].motor_deg)
        {
            height_mm = PendulumGeometry_Interpolate(magnitude,
                                                     s_points[index - 1U].motor_deg,
                                                     s_points[index - 1U].end_height_mm,
                                                     s_points[index].motor_deg,
                                                     s_points[index].end_height_mm);
            return PendulumGeometry_Sign(motor_angle_deg) * height_mm;
        }
    }

    /* 18..25 度没有实测点。仅用最后一段
     * 实测斜率外推，同时硬性 25 度安全限幅保持生效。 */
    height_mm = PendulumGeometry_Interpolate(magnitude,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 2U].motor_deg,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 2U].end_height_mm,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 1U].motor_deg,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 1U].end_height_mm);
    return PendulumGeometry_Sign(motor_angle_deg) * height_mm;
}

float PendulumGeometry_MotorDegToBeamDeg(float motor_angle_deg)
{
    float ratio = PendulumGeometry_MotorDegToEndHeightMm(motor_angle_deg) / Q3_BEAM_LENGTH_MM;

    if (ratio > 1.0f)
        ratio = 1.0f;
    if (ratio < -1.0f)
        ratio = -1.0f;
    return asinf(ratio) * Q3_RAD_TO_DEG;
}

bool PendulumGeometry_BeamDegToMotorDeg(float beam_angle_deg, float *motor_angle_deg)
{
    float requested_height_mm;
    float magnitude;
    float sign;
    float motor_deg;
    size_t index;

    if (motor_angle_deg == NULL)
    {
        return false;
    }

    sign = PendulumGeometry_Sign(beam_angle_deg);
    /* 控制器只请求较小的横梁角度。保留此本地
     * 五阶正弦近似，使 MSPM0 构建不依赖
     * 庞大的 libm sinf 实现。3 度以下误差可忽略不计。 */
    {
        float angle_rad = PendulumGeometry_Abs(beam_angle_deg) * Q3_DEG_TO_RAD;
        float angle_sq = angle_rad * angle_rad;
        float sine = angle_rad * (1.0f - angle_sq / 6.0f + (angle_sq * angle_sq) / 120.0f);
        requested_height_mm = Q3_BEAM_LENGTH_MM * sine;
    }
    if (requested_height_mm >
        PendulumGeometry_Abs(PendulumGeometry_MotorDegToEndHeightMm(Q3_MOTOR_SAFE_LIMIT_DEG)))
    {
        return false;
    }

    magnitude = requested_height_mm;
    for (index = 1U; index < Q3_GEOMETRY_POINT_COUNT; ++index)
    {
        if (magnitude <= s_points[index].end_height_mm)
        {
            motor_deg = PendulumGeometry_Interpolate(magnitude,
                                                     s_points[index - 1U].end_height_mm,
                                                     s_points[index - 1U].motor_deg,
                                                     s_points[index].end_height_mm,
                                                     s_points[index].motor_deg);
            *motor_angle_deg = sign * motor_deg;
            return true;
        }
    }

    motor_deg = PendulumGeometry_Interpolate(magnitude,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 2U].end_height_mm,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 2U].motor_deg,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 1U].end_height_mm,
                                             s_points[Q3_GEOMETRY_POINT_COUNT - 1U].motor_deg);
    if (motor_deg > Q3_MOTOR_SAFE_LIMIT_DEG)
    {
        return false;
    }
    *motor_angle_deg = sign * motor_deg;
    return true;
}

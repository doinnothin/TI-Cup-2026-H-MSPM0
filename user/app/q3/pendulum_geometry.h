#ifndef PENDULUM_GEOMETRY_H
#define PENDULUM_GEOMETRY_H

#include <stdbool.h>

/* 将电机角度标定转换为物理横梁几何。
 * 保留符号：正电机角度表示测得的右端
 * 高于其零位。 */
float PendulumGeometry_MotorDegToEndHeightMm(float motor_angle_deg);
float PendulumGeometry_MotorDegToBeamDeg(float motor_angle_deg);
/* 仅限实测 0..18 deg 表。负侧按奇对称处理。 */
bool PendulumGeometry_EndHeightMmToMotorDeg(float height_mm, float *motor_deg);

/* 供后续前馈/模型使用的逆查表。当所请求的
 * 横梁角度超出实测机械安全包络时返回 false。 */
bool PendulumGeometry_BeamDegToMotorDeg(float beam_angle_deg, float *motor_angle_deg);

#endif

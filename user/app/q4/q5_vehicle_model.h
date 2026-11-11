#ifndef Q5_VEHICLE_MODEL_H
#define Q5_VEHICLE_MODEL_H

#include <stdbool.h>
#include <stdint.h>

/* 纯调用方持有的 Q5 车辆运动模型。速度单位为 mm/s，距离为 cm，
 * 加速度为 m/s^2，弯道混合量无量纲，时间戳为 ms。 */
typedef struct
{
    uint32_t sampled_ms;
    float speed_mm_s;
    float longitudinal_accel_m_s2;
    float lateral_accel_m_s2;
    float parallel_accel_m_s2;
    float turn_blend;
    bool initialized;
} Q5VehicleModel;

void Q5VehicleModel_Init(Q5VehicleModel *model);
/* 根据一个前台采样更新。返回 false 表示输入无效，或
 * 配置的最小采样间隔尚未到。 */
bool Q5VehicleModel_Update(
    Q5VehicleModel *model, uint32_t now_ms, float left_mm_s, float right_mm_s, float distance_cm);
/* 使用定稿的 Q5 路线成形横向前馈进行更新。原有
 * 入口仍保留，供有意沿用其规律的模式使用。 */
bool Q5VehicleModel_UpdateFinalQ5(
    Q5VehicleModel *model, uint32_t now_ms, float left_mm_s, float right_mm_s, float distance_cm);
/* 确定性路线 profile；返回以 mm/s 为单位的速度与 [-1,1] 内的带符号混合量。 */
float Q5VehicleModel_TraceSpeed(uint32_t elapsed_ms);
float Q5VehicleModel_TurnBlend(float distance_cm, float speed_mm_s);

#endif

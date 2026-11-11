#ifndef LAP_TRACE_H
#define LAP_TRACE_H

#include <stdint.h>

/* Q2 单圈距离/时间状态机。启动/复位为前台操作；
 * LapTrace_Tick10ms 由现有的 10 ms 定时器 ISR 调用。距离
 * 单位为 cm，经过/限制时间单位为 ms。与 ISR 共享的状态并非任务队列。
 */

/* 第二问：八路灰度循迹一圈。速度单位与 E_speed 一致，均为 mm/s。 */
#define LAP_TRACE_TARGET_SPEED 400.0f

/* 图纸一圈中心线：2×1.5 m + 2π×0.5 m = 614.16 cm。 */
#define LAP_TRACE_LENGTH_CM 584.16f
/* 实车停车超过 A 线就增大本值，停在 A 线前就减小本值。 */
#define LAP_TRACE_BRAKE_COMP_CM 3.00f
#define LAP_TRACE_STOP_DISTANCE_CM (LAP_TRACE_LENGTH_CM - LAP_TRACE_BRAKE_COMP_CM)
#define LAP_TRACE_TIME_LIMIT_MS 20000UL

typedef enum
{
    LAP_TRACE_IDLE = 0,
    LAP_TRACE_RUNNING,
    LAP_TRACE_FINISHED,
    LAP_TRACE_TIMEOUT
} LapTraceState_t;

/* 将状态机复位到 IDLE 并恢复配置默认值。 */
void LapTrace_Reset(void);
void LapTrace_Start(void);
/* 以显式限制启动；非正距离或时间为零时选用
 * 模块默认值，与现有 Q2 调用契约一致。 */
void LapTrace_StartToDistance(float stop_distance_cm, uint32_t time_limit_ms);
/* 在中断上下文中推进一个 10 ms 调度步。 */
void LapTrace_Tick10ms(void);

uint32_t LapTrace_GetElapsedMs(void);
LapTraceState_t LapTrace_GetState(void);
/* 当终端状态请求底盘制动时返回 1。 */
uint8_t LapTrace_StopRequested(void);

#endif

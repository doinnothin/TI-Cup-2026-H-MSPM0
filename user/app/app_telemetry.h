#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdint.h>

/*
 * 既有周期性诊断记录的前台调度器。
 * BeginCycle 在视觉/控制工作前快照当前 Q6 运行状态，以便
 * 各模式的周期与先前的主循环时序一致。PublishDue 在同一循环中
 * 稍后运行（在 DebugLog_Task 之后），仅将到期的记录入队；它绝不
 * 直接发送。时间戳为单调递增的毫秒。切勿从
 * ISR 中调用。
 */
void AppTelemetry_BeginCycle(void);
void AppTelemetry_PublishDue(uint32_t now_ms);

#endif

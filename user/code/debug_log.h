#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdint.h>

/*
 * 固定缓冲区调试遥测序列化器与 UART1 发送队列。Publish 调用会快照模块状态、
 * 按现有线缆记录格式排版并入队；队列满时丢弃新记录。DebugLog_Task 每次前台
 * 调用排空有限数量的字节，不会等待 UART 空间。所有 API 仅限前台使用：格式化、
 * 状态快照和队列修改均非 ISR 安全。时间戳单位为毫秒；载荷单位与字段顺序
 * 属于既有日志协议的一部分，必须保持稳定。
 */

void DebugLog_Init(void);
void DebugLog_PublishQ6Event(uint32_t now_ms, uint8_t code, int32_t value);
void DebugLog_PublishVision(uint32_t now_ms);
void DebugLog_PublishActuator(uint32_t now_ms);
void DebugLog_PublishPendulumCalibration(uint32_t now_ms);
void DebugLog_PublishBallPositionControl(uint32_t now_ms);
void DebugLog_PublishBallMechanismId(uint32_t now_ms);
void DebugLog_PublishPendulumGeometryCalibration(uint32_t now_ms);
void DebugLog_PublishBallPulseId(uint32_t now_ms);
void DebugLog_PublishQ3Sequence(uint32_t now_ms);
void DebugLog_PublishQ45Vehicle(uint32_t now_ms);
void DebugLog_PublishControlFrame(uint32_t now_ms);
void DebugLog_Task(void);

#endif

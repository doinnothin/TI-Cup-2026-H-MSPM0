#ifndef Q6_TARGET_H
#define Q6_TARGET_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Q6 稳定目标选择与捕获逻辑；本模块不驱动
 * 硬件。位置单位为 cm，速度单位为 cm/s，时间戳单位为 ms。请在主循环中调用
 * Init/Update/Lock/GetStatus，而非在 ISR 中。Update 消费
 * 一份调用者提供的帧快照；调用者不得以陈旧的
 * 缓存视觉数据替代新帧。在候选窗口满足所配置的
 * 新鲜度/稳定性判据之前，Lock 返回 false。
 */

typedef enum
{
    Q6_TARGET_EXPLICIT = 0,
    Q6_TARGET_CAPTURE = 1
} Q6TargetSource;

typedef struct
{
    float position_cm;
    float velocity_cm_s;
    uint32_t rx_time_ms;
    uint16_t source_timestamp_ms;
    uint8_t confidence_pct;
    uint8_t sequence;
    bool valid;
} Q6TargetFrame;

typedef struct
{
    Q6TargetSource source;
    float requested_cm;
    float candidate_cm;
    float locked_cm;
    uint32_t independent_frames;
    uint32_t span_ms;
    bool ready;
    bool locked;
    uint32_t rejected_frames;
} Q6TargetStatus;

/* 复位捕获窗口。explicit_target_cm 仅在 EXPLICIT 模式下使用。 */
void Q6Target_Init(Q6TargetSource source, float explicit_target_cm);
/* Q6 视觉准入策略：接受位于 Q6_BALL_PIXEL_CY_CENTER 周围、所配置的
 * 闭区间像素带内的行。模式选择仍由调用者负责。 */
bool Q6Target_IsPixelRowAccepted(uint16_t pixel_cy);
/* 消费一份新帧快照；无效、陈旧、重放或不稳定的
 * 样本无法推进锁就绪窗口。 */
void Q6Target_Update(const Q6TargetFrame *sample, uint32_t now_ms);
/* 成功时锁定一次并写入 target_cm；返回 false 表示尚未就绪、已
 * 锁定，或输出指针为空。 */
bool Q6Target_Lock(float *target_cm);
/* 当 status 非空时复制当前模块快照。 */
void Q6Target_GetStatus(Q6TargetStatus *status);

#endif

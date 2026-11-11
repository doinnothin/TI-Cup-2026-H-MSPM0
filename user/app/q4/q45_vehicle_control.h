#ifndef Q45_VEHICLE_CONTROL_H
#define Q45_VEHICLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "app/q6/q6_target.h"

/*
 * Q4/Q5/Q6 共享的车辆任务与小球控制适配层。
 *
 * 单位：API 时间戳为毫秒；位置与目标值为 cm；速度单位为 cm/s；
 * 执行器角度单位为度。调用方需为状态快照提供有效的输出指针。
 * Init/SelectMode/Start/Task/Stop 均在主循环上下文运行。
 * Q45Vehicle_Tick10msISR 是唯一的中断入口，必须保持有界，且不得进行
 * UART 发送或阻塞调用。Task 与模式选择调用可下发执行器/底盘命令。
 */

typedef enum
{
    Q45_VEHICLE_MODE_NONE = 0,
    Q45_VEHICLE_MODE_Q4,
    Q45_VEHICLE_MODE_Q5,
    Q45_VEHICLE_MODE_Q6
} Q45VehicleMode;

typedef enum
{
    Q45_VEHICLE_IDLE = 0,
    Q45_VEHICLE_WAIT_ACTUATOR,
    Q45_VEHICLE_CONFIGURE,
    Q45_VEHICLE_ZERO,
    /* 所选模式的执行器调零序列已完成。Q4/Q5 标记当前姿态；Q6 返回其
     * 存储的绝对原点。Q4/Q5 等待启动请求；常规 Q6 按键请求在目标捕获
     * 期间保持挂起，并在达到就绪时启动。 */
    Q45_VEHICLE_READY,
    Q45_VEHICLE_RUNNING,
    Q45_VEHICLE_FINISHED,
    Q45_VEHICLE_TIMEOUT,
    Q45_VEHICLE_VISION_LOST,
    Q45_VEHICLE_FAULT,
    Q45_VEHICLE_STOPPED
} Q45VehicleState;

typedef struct
{
    Q45VehicleState state;
    Q45VehicleMode mode;
    uint32_t state_started_ms;
    uint32_t run_sequence;
    uint32_t elapsed_ms;       /* 正式运行时间，从成功调用 StartSelected() 起算。 */
    uint32_t drive_elapsed_ms; /* 从首条底盘命令起算。 */
    uint32_t pass_elapsed_ms;  /* Q4 的 B 点，Q5 的 A 点。 */
    float route_distance_cm;
    float base_speed_mm_s;
    float actual_speed_mm_s;
    float longitudinal_accel_m_s2;
    float modeled_turn_rate_deg_s;
    float lateral_accel_m_s2;
    float parallel_accel_m_s2;
    float raw_position_cm;
    float measured_cm;
    float measured_velocity_cm_s;
    float predicted_position_cm;
    /* 相对正式运行起始采样点的峰值绝对位移；在进入终止状态后仍保留，
     * 用于结果报告。 */
    float max_abs_position_cm;
    float error_cm;
    float control_error_cm;
    float p_beam_deg;
    float d_beam_deg;
    float i_beam_deg;
    float feedforward_beam_deg;
    float requested_beam_deg;
    float control_output_deg;
    float commanded_angle_deg;
    float actual_angle_deg;
    uint8_t control_profile;
    uint8_t stiction_count;
    uint8_t speed_governor_level;
    uint8_t vision_confidence_pct;
    uint8_t estimator_rejected_count;
    uint8_t vision_sequence;
    uint32_t vision_age_ms;
    uint32_t accepted_age_ms;
    uint32_t actual_angle_age_ms;
    uint32_t max_frame_gap_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    uint8_t stop_reason;
    bool route_passed;
    bool center_ready;
    bool in_scoring_band;
    bool band_violation;
    bool control_output_limited;
    bool terminal_zero_confirmed;
    bool actuator_paused;
    bool acceleration_feedforward_enabled;
    bool stop_requested;
    bool fault;
    /* 仅 Q6：锁定的保持目标及其峰值原始偏差。Q4/Q5 保持 target_cm 为 0，
     * 且从不置位 target_locked。 */
    float target_cm;
    float max_target_dev_cm;
    bool target_locked;
} Q45VehicleStatus;

/* 初始化软件状态；在选择车辆模式前调用一次。 */
void Q45Vehicle_Init(void);
/* 仅在未激活时选择 Q4/Q5/Q6，并进入 WAIT_ACTUATOR。
 * now_ms 为单调毫秒时间戳；返回 false 表示模式被拒绝或已有运行在进行。 */
bool Q45Vehicle_SelectMode(Q45VehicleMode mode, uint32_t now_ms);
/* 在选择 Q6 之前配置 Q6 目标源。激活期间、目标源不受支持、或显式目标
 * 非有限/超范围时拒绝。 */
bool Q45Vehicle_Q6ConfigureTarget(Q6TargetSource source, float explicit_cm);
/* 在中心、执行器调零以及（对 Q6）目标锁定等前置条件满足后，从 READY
 * 启动正式计时/控制。可能下发硬件命令。 */
bool Q45Vehicle_StartSelected(uint32_t now_ms);
/* 推进前台状态/控制逻辑；需由主循环定期调用。
 * 可能下发执行器 UART 与底盘命令。 */
void Q45Vehicle_Task(uint32_t now_ms);
/* 仅由现有 10 ms 底盘定时器中断调用。仅做有界的簿记
 * 与速度更新；不得执行 UART 发送。 */
void Q45Vehicle_Tick10msISR(void);
/* 在前台上下文请求操作员停止；非激活模式被忽略。 */
void Q45Vehicle_Stop(uint32_t now_ms);
/* 只读状态查询；StopRequested 会保持置位，直至重新初始化或
 * 下一次模式选择。 */
bool Q45Vehicle_IsActive(void);
bool Q45Vehicle_StopRequested(void);
/* 返回并清除一次性的终止结果记录请求。 */
bool Q45Vehicle_ConsumeResultLog(void);
/* 当 status 非空时复制一份状态快照。 */
void Q45Vehicle_GetStatus(Q45VehicleStatus *status);
/* 为枚举值返回稳定、以空字符结尾的显示标签。 */
const char *Q45Vehicle_StateName(Q45VehicleState state);
const char *Q45Vehicle_ModeName(Q45VehicleMode mode);

#endif

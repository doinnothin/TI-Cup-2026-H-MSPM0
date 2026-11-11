#ifndef Q4_STATIC_RECOVERY_H
#define Q4_STATIC_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>

/* 仅作归档调参用途。比赛用的 Q3/Q4/Q5 镜像
 * 绝不能构建或运行该自主随机扰动测试状态机。 */
#define Q4_STATIC_RECOVERY_TEST_ENABLED 0U

typedef enum
{
    Q4_STATIC_WAIT = 0,
    Q4_STATIC_DISTURB,
    Q4_STATIC_RECOVER,
    Q4_STATIC_HOLD,
    Q4_STATIC_RETURN_ZERO,
    Q4_STATIC_FINISH,
    Q4_STATIC_FAULT
} Q4StaticState;

typedef struct
{
    Q4StaticState state;
    uint32_t state_started_ms;
    uint32_t recovery_elapsed_ms;
    uint32_t run_sequence;
    float disturbance_angle_deg;
    float target_cm;
    float measured_cm;
    float measured_velocity_cm_s;
    float error_cm;
    float control_error_cm;
    float control_output_deg;
    float commanded_angle_deg;
    float actual_angle_deg;
    uint8_t control_profile;
    float recovery_limit_deg;
    uint8_t vision_sequence;
    uint32_t vision_age_ms;
    bool target_reached;
    bool acceptance_passed;
    uint8_t breakaway_count;
    uint8_t fault_reason;
    bool fault;
} Q4StaticStatus;

void Q4StaticRecovery_Init(void);
void Q4StaticRecovery_Task(uint32_t now_ms);
void Q4StaticRecovery_Stop(uint32_t now_ms);
bool Q4StaticRecovery_IsActive(void);
bool Q4StaticRecovery_ConsumeResultLog(void);
void Q4StaticRecovery_GetStatus(Q4StaticStatus *status);
const char *Q4StaticRecovery_StateName(Q4StaticState state);

#endif

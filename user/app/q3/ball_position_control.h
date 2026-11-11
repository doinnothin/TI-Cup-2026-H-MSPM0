#ifndef BALL_POSITION_CONTROL_H
#define BALL_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "ball_outer_pd.h"

typedef enum
{
    BALL_CONTROL_IDLE = 0,
    BALL_CONTROL_PRECHECK,
    BALL_CONTROL_ENABLE_SETTLE,
    BALL_CONTROL_ZERO_GAP,
    BALL_CONTROL_KICK_PARAM_GAP,
    BALL_CONTROL_KICK,
    BALL_CONTROL_RETURN,
    BALL_CONTROL_NORMAL_PARAM_GAP,
    BALL_CONTROL_ACTIVE,
    BALL_CONTROL_HOLD,
    BALL_CONTROL_FAULT,
    BALL_CONTROL_STOPPED
} BallPositionControlState;

typedef enum
{
    BALL_CONTROL_FAULT_NONE = 0,
    BALL_CONTROL_FAULT_ACTUATOR_NOT_READY,
    BALL_CONTROL_FAULT_VISION_STALE,
    BALL_CONTROL_FAULT_ENABLE_COMMAND,
    BALL_CONTROL_FAULT_ZERO_COMMAND,
    BALL_CONTROL_FAULT_PARAMETER_COMMAND,
    BALL_CONTROL_FAULT_POSITION_COMMAND,
    BALL_CONTROL_FAULT_START_POSITION,
    BALL_CONTROL_FAULT_ANGLE_STALE,
    BALL_CONTROL_FAULT_KICK_TIMEOUT,
    BALL_CONTROL_FAULT_RETURN_TIMEOUT,
    BALL_CONTROL_FAULT_SAFETY_POSITION,
    BALL_CONTROL_FAULT_RUN_TIMEOUT,
    BALL_CONTROL_FAULT_TRACKING,
    BALL_CONTROL_FAULT_VISION_TIME
} BallPositionControlFault;

typedef struct
{
    BallPositionControlState state;
    BallPositionControlFault fault;
    float target_cm;
    float measured_cm;
    float error_cm;
    float control_output_deg;
    float target_angle_deg;
    float actual_angle_deg;
    float kick_peak_angle_deg;
    float measured_velocity_cm_s;
    float vision_velocity_cm_s;
    float pid_p_deg;
    float pid_i_deg;
    float pid_d_deg;
    float stopping_distance_cm;
    BallOuterPdOutput outer_pd;
    float estimator_innovation_cm;
    uint8_t estimator_reject_count;
    uint32_t pid_updates;
    uint32_t command_count;
    uint32_t sample_dt_ms;
    uint32_t dispatch_age_ms;
    bool braking;
    uint8_t vision_sequence;
    uint32_t vision_age_ms;
    uint32_t actual_angle_age_ms;
    uint32_t kick_elapsed_ms;
    uint32_t state_started_ms;
    uint32_t run_sequence;
    uint8_t rekick_count;
    bool vision_fresh;
    bool actual_angle_fresh;
    bool target_reached;
} BallPositionControlStatus;

void BallPositionControl_Init(void);
bool BallPositionControl_Start(uint32_t now_ms);
/* 仅限安全内部起始；target 为 cm，与电机零点无关。 */
bool BallPositionControl_StartTargetCm(float target_cm, uint32_t now_ms);
/* 改变 cm 目标，但不丢弃已测速度、也不强制冲击。 */
bool BallPositionControl_SetTargetCm(float target_cm, uint32_t now_ms);
void BallPositionControl_SetOuterProfile(BallOuterPdProfile profile);
/* 运行结束动作：将 X42S 指令到其当前会话的 0 度
 * 目标，而不调用持久的绝对零点返回指令。 */
bool BallPositionControl_ReturnMotorToZero(uint32_t now_ms);
bool BallPositionControl_IsMotorAtZero(void);
void BallPositionControl_Stop(uint32_t now_ms);
void BallPositionControl_Task(uint32_t now_ms);
bool BallPositionControl_IsActive(void);
bool BallPositionControl_IsArrived(void);
bool BallPositionControl_HasFault(void);
bool BallPositionControl_ConsumeResultLog(void);
void BallPositionControl_GetStatus(BallPositionControlStatus *status);
const char *BallPositionControl_StateName(BallPositionControlState state);

#endif

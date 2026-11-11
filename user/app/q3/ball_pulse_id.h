#ifndef BALL_PULSE_ID_H
#define BALL_PULSE_ID_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BALL_PULSE_IDLE = 0,
    BALL_PULSE_PRECHECK,
    BALL_PULSE_ENABLE_SETTLE,
    BALL_PULSE_ZERO_GAP,
    BALL_PULSE_PARAM_GAP,
    BALL_PULSE_ZERO_SETTLE,
    BALL_PULSE_WAIT_TRIGGER,
    BALL_PULSE_RISE,
    BALL_PULSE_HOLD,
    BALL_PULSE_RETURN,
    BALL_PULSE_RESULT_SETTLE,
    BALL_PULSE_SAFETY_RETURN,
    BALL_PULSE_DONE,
    BALL_PULSE_FAULT,
    BALL_PULSE_ABORTED
} BallPulseIdState;

typedef enum
{
    BALL_PULSE_FAULT_NONE = 0,
    BALL_PULSE_FAULT_ACTUATOR_NOT_READY,
    BALL_PULSE_FAULT_VISION_STALE,
    BALL_PULSE_FAULT_ENABLE_COMMAND,
    BALL_PULSE_FAULT_ZERO_COMMAND,
    BALL_PULSE_FAULT_PARAMETER_COMMAND,
    BALL_PULSE_FAULT_POSITION_COMMAND,
    BALL_PULSE_FAULT_START_NOT_CENTERED
} BallPulseIdFault;

typedef struct
{
    BallPulseIdState state;
    BallPulseIdFault fault;
    float target_angle_deg;
    float start_cm;
    float peak_cm;
    float end_cm;
    float peak_velocity_cm_s;
    uint8_t vision_sequence;
    uint32_t vision_age_ms;
    uint32_t state_started_ms;
    bool vision_fresh;
    bool safety_return;
} BallPulseIdStatus;

void BallPulseId_Init(void);
/* 第一次按 PA31：使能，建立标记的零位姿态，指令 0 deg。 */
bool BallPulseId_Start(uint32_t now_ms);
/* 第二次按 PA31：恰好一个 -6.5 deg -> 0 deg 脉冲。 */
bool BallPulseId_Trigger(uint32_t now_ms);
void BallPulseId_Abort(uint32_t now_ms);
void BallPulseId_Task(uint32_t now_ms);
bool BallPulseId_IsActive(void);
bool BallPulseId_IsWaitingForTrigger(void);
bool BallPulseId_ConsumeResultLog(void);
void BallPulseId_GetStatus(BallPulseIdStatus *status);
const char *BallPulseId_StateName(BallPulseIdState state);

#endif

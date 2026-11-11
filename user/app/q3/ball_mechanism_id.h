#ifndef BALL_MECHANISM_ID_H
#define BALL_MECHANISM_ID_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BALL_ID_IDLE = 0,
    BALL_ID_PRECHECK,
    BALL_ID_ENABLE_SETTLE,
    BALL_ID_ZERO_GAP,
    BALL_ID_PARAM_GAP,
    BALL_ID_WAIT_POINT,
    BALL_ID_HOLD_POINT,
    BALL_ID_RETURN_SETTLE,
    BALL_ID_WAIT_NEGATIVE,
    BALL_ID_COMPLETE,
    BALL_ID_FAULT,
    BALL_ID_ABORTED
} BallMechanismIdState;

typedef enum
{
    BALL_ID_FAULT_NONE = 0,
    BALL_ID_FAULT_ACTUATOR_NOT_READY,
    BALL_ID_FAULT_VISION_STALE,
    BALL_ID_FAULT_ENABLE_COMMAND,
    BALL_ID_FAULT_ZERO_COMMAND,
    BALL_ID_FAULT_PARAMETER_COMMAND,
    BALL_ID_FAULT_POSITION_COMMAND
} BallMechanismIdFault;

typedef struct
{
    BallMechanismIdState state;
    BallMechanismIdFault fault;
    int8_t direction;
    float next_angle_deg;
    float target_angle_deg;
    float expected_start_cm;
    float current_cm;
    float start_cm;
    float end_cm;
    float delta_cm;
    float positive_safe_deg;
    float negative_safe_deg;
    uint32_t point_sequence;
    uint32_t state_started_ms;
    uint32_t vision_age_ms;
    uint16_t trigger_rejects;
    bool moved;
    bool vision_fresh;
} BallMechanismIdStatus;

void BallMechanismId_Init(void);
bool BallMechanismId_Start(uint32_t now_ms);
/* 仅当状态机等待操作员把球放到所请求的 +/-5 cm 起始点时，
 * 由 PA31 调用。 */
bool BallMechanismId_TriggerPoint(uint32_t now_ms);
void BallMechanismId_Abort(uint32_t now_ms);
void BallMechanismId_Task(uint32_t now_ms);
bool BallMechanismId_IsActive(void);
bool BallMechanismId_IsWaitingForTrigger(void);
bool BallMechanismId_ConsumeResultLog(void);
void BallMechanismId_GetStatus(BallMechanismIdStatus *status);
const char *BallMechanismId_StateName(BallMechanismIdState state);

#endif

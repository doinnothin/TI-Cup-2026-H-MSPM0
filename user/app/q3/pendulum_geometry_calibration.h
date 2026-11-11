#ifndef PENDULUM_GEOMETRY_CALIBRATION_H
#define PENDULUM_GEOMETRY_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PENDULUM_GEO_IDLE = 0,
    PENDULUM_GEO_PRECHECK,
    PENDULUM_GEO_ENABLE_SETTLE,
    PENDULUM_GEO_ZERO_GAP,
    PENDULUM_GEO_PARAM_GAP,
    PENDULUM_GEO_ZERO_SETTLE,
    PENDULUM_GEO_WAIT_POINT,
    PENDULUM_GEO_LIMIT,
    PENDULUM_GEO_FAULT,
    PENDULUM_GEO_ABORTED
} PendulumGeometryCalibrationState;

typedef enum
{
    PENDULUM_GEO_FAULT_NONE = 0,
    PENDULUM_GEO_FAULT_ACTUATOR_NOT_READY,
    PENDULUM_GEO_FAULT_ENABLE_COMMAND,
    PENDULUM_GEO_FAULT_ZERO_COMMAND,
    PENDULUM_GEO_FAULT_PARAMETER_COMMAND,
    PENDULUM_GEO_FAULT_POSITION_COMMAND
} PendulumGeometryCalibrationFault;

typedef struct
{
    PendulumGeometryCalibrationState state;
    PendulumGeometryCalibrationFault fault;
    float target_angle_deg;
    float next_angle_deg;
    uint32_t point_sequence;
    uint32_t state_started_ms;
} PendulumGeometryCalibrationStatus;

void PendulumGeometryCalibration_Init(void);
/* 仅第一次按 PA31：建立参考并指令 0 deg。 */
bool PendulumGeometryCalibration_Start(uint32_t now_ms);
/* 后续按 PA31：0 -> +2 -> +4 …… 中间不返回。 */
bool PendulumGeometryCalibration_Advance(uint32_t now_ms);
void PendulumGeometryCalibration_Abort(uint32_t now_ms);
void PendulumGeometryCalibration_Task(uint32_t now_ms);
bool PendulumGeometryCalibration_IsActive(void);
bool PendulumGeometryCalibration_IsWaitingForAdvance(void);
bool PendulumGeometryCalibration_ConsumeResultLog(void);
void PendulumGeometryCalibration_GetStatus(PendulumGeometryCalibrationStatus *status);
const char *PendulumGeometryCalibration_StateName(PendulumGeometryCalibrationState state);

#endif

#ifndef PENDULUM_CALIBRATION_H
#define PENDULUM_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

/* 用于摆杆角度标定的前台诊断状态机。它不持有
 * 任何 UART 驱动状态；命令与反馈访问均通过 BallActuator。
 * Task/Start/Abort 可能下发电机命令，绝不能运行在 ISR 中。
 */
typedef enum
{
    PENDULUM_CAL_IDLE = 0,
    PENDULUM_CAL_PRECHECK,
    PENDULUM_CAL_ENABLE_SETTLE,
    PENDULUM_CAL_ZERO_GAP,
    PENDULUM_CAL_PARAM_GAP,
    PENDULUM_CAL_POS_HOLD,
    PENDULUM_CAL_ZERO1_HOLD,
    PENDULUM_CAL_NEG_HOLD,
    PENDULUM_CAL_ZERO2_HOLD,
    PENDULUM_CAL_STOP_GAP,
    PENDULUM_CAL_DISABLE,
    PENDULUM_CAL_COMPLETE,
    PENDULUM_CAL_FAULT,
    PENDULUM_CAL_ABORTED
} PendulumCalibrationState;

typedef enum
{
    PENDULUM_CAL_FAULT_NONE = 0,
    PENDULUM_CAL_FAULT_ACTUATOR_OFFLINE,
    PENDULUM_CAL_FAULT_ENABLE_COMMAND,
    PENDULUM_CAL_FAULT_ZERO_COMMAND,
    PENDULUM_CAL_FAULT_PARAMETER_COMMAND,
    PENDULUM_CAL_FAULT_POSITION_COMMAND,
    PENDULUM_CAL_FAULT_TOTAL_TIMEOUT
} PendulumCalibrationFault;

typedef struct
{
    PendulumCalibrationState state;
    PendulumCalibrationFault fault;
    float target_angle_deg;
    uint32_t sequence;
    uint32_t state_started_ms;
} PendulumCalibrationStatus;

void PendulumCalibration_Init(void);
/* 启动一次标定序列；若已有序列处于活动状态则返回 false。 */
bool PendulumCalibration_Start(uint32_t now_ms);
void PendulumCalibration_Abort(uint32_t now_ms);
void PendulumCalibration_Task(uint32_t now_ms);
bool PendulumCalibration_IsActive(void);
bool PendulumCalibration_HasResult(void);
bool PendulumCalibration_ConsumeResultLog(void);
void PendulumCalibration_GetStatus(PendulumCalibrationStatus *status);
const char *PendulumCalibration_StateName(PendulumCalibrationState state);

#endif

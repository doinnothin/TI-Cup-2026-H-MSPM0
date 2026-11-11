#include "pendulum_geometry_calibration.h"

#include <string.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/pendulum_geometry_calibration_config.h"

static PendulumGeometryCalibrationStatus s_status;
static bool s_result_log_pending;

static bool PendulumGeometryCalibration_Expired(uint32_t now_ms, uint32_t interval_ms)
{
    return (now_ms - s_status.state_started_ms) >= interval_ms;
}

static void PendulumGeometryCalibration_Enter(PendulumGeometryCalibrationState state,
                                              uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
    if ((state == PENDULUM_GEO_WAIT_POINT) || (state == PENDULUM_GEO_LIMIT) ||
        (state == PENDULUM_GEO_FAULT) || (state == PENDULUM_GEO_ABORTED))
    {
        s_result_log_pending = true;
    }
}

static void PendulumGeometryCalibration_Fail(PendulumGeometryCalibrationFault fault,
                                             uint32_t now_ms)
{
    s_status.fault = fault;
    (void)BallActuator_Stop();
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    PendulumGeometryCalibration_Enter(PENDULUM_GEO_FAULT, now_ms);
}

void PendulumGeometryCalibration_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = PENDULUM_GEO_IDLE;
    s_result_log_pending = false;
}

bool PendulumGeometryCalibration_Start(uint32_t now_ms)
{
    if (PendulumGeometryCalibration_IsActive())
    {
        return false;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_status.state = PENDULUM_GEO_IDLE;
    s_status.target_angle_deg = PENDULUM_GEO_FIRST_ANGLE_DEG;
    s_status.next_angle_deg = PENDULUM_GEO_FIRST_ANGLE_DEG + PENDULUM_GEO_ANGLE_STEP_DEG;
    s_status.fault = PENDULUM_GEO_FAULT_NONE;
    s_result_log_pending = false;
    PendulumGeometryCalibration_Enter(PENDULUM_GEO_PRECHECK, now_ms);
    return true;
}

bool PendulumGeometryCalibration_Advance(uint32_t now_ms)
{
    float requested_angle;

    if (s_status.state != PENDULUM_GEO_WAIT_POINT)
    {
        return false;
    }

    requested_angle = s_status.next_angle_deg;
    if (requested_angle > PENDULUM_GEO_MAX_TEST_ANGLE_DEG)
    {
        PendulumGeometryCalibration_Enter(PENDULUM_GEO_LIMIT, now_ms);
        return false;
    }

    if (!BallActuator_SetAbsoluteAngleDeg(requested_angle))
    {
        PendulumGeometryCalibration_Fail(PENDULUM_GEO_FAULT_POSITION_COMMAND, now_ms);
        return false;
    }

    s_status.target_angle_deg = requested_angle;
    s_status.next_angle_deg = requested_angle + PENDULUM_GEO_ANGLE_STEP_DEG;
    ++s_status.point_sequence;
    PendulumGeometryCalibration_Enter(PENDULUM_GEO_WAIT_POINT, now_ms);
    return true;
}

void PendulumGeometryCalibration_Abort(uint32_t now_ms)
{
    if (PendulumGeometryCalibration_IsActive())
    {
        (void)BallActuator_Stop();
        (void)BallActuator_Enable(false);
        BallActuator_SetCommandSession(false);
        PendulumGeometryCalibration_Enter(PENDULUM_GEO_ABORTED, now_ms);
    }
}

bool PendulumGeometryCalibration_IsActive(void)
{
    return (s_status.state >= PENDULUM_GEO_PRECHECK) && (s_status.state <= PENDULUM_GEO_LIMIT);
}

bool PendulumGeometryCalibration_IsWaitingForAdvance(void)
{
    return s_status.state == PENDULUM_GEO_WAIT_POINT;
}

bool PendulumGeometryCalibration_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void PendulumGeometryCalibration_Task(uint32_t now_ms)
{
    BallActuatorStatus actuator;

    if (!PendulumGeometryCalibration_IsActive())
    {
        return;
    }

    BallActuator_GetStatus(&actuator);
    switch (s_status.state)
    {
    case PENDULUM_GEO_PRECHECK:
        if (!actuator.online || !actuator.firmware_x || !actuator.foc_closed_loop)
        {
            PendulumGeometryCalibration_Fail(PENDULUM_GEO_FAULT_ACTUATOR_NOT_READY, now_ms);
        }
        else
        {
            /* 保持选项轮询，直到 X42S 证明其已
                 * 在线；只有此时才为本序列保留 UART 通信。 */
            BallActuator_SetCommandSession(true);
            if (!BallActuator_Enable(true))
            {
                PendulumGeometryCalibration_Fail(PENDULUM_GEO_FAULT_ENABLE_COMMAND, now_ms);
            }
            else
            {
                PendulumGeometryCalibration_Enter(PENDULUM_GEO_ENABLE_SETTLE, now_ms);
            }
        }
        break;

    case PENDULUM_GEO_ENABLE_SETTLE:
        if (PendulumGeometryCalibration_Expired(now_ms, PENDULUM_GEO_ENABLE_SETTLE_MS))
        {
            /* 操作员在第一次按 PA31 前把摆臂放到
                 * 标记的机械零位。此指令把该物理姿态
                 * 确立为 X42S 软件角度 0 deg。 */
            if (!BallActuator_ZeroAtCurrentPosition())
            {
                PendulumGeometryCalibration_Fail(PENDULUM_GEO_FAULT_ZERO_COMMAND, now_ms);
            }
            else
            {
                PendulumGeometryCalibration_Enter(PENDULUM_GEO_ZERO_GAP, now_ms);
            }
        }
        break;

    case PENDULUM_GEO_ZERO_GAP:
        if (PendulumGeometryCalibration_Expired(now_ms, PENDULUM_GEO_COMMAND_GAP_MS))
        {
            if (!BallActuator_PrepareAbsolutePosition(PENDULUM_GEO_ACCEL_RPM_S,
                                                      PENDULUM_GEO_DECEL_RPM_S,
                                                      PENDULUM_GEO_MAX_SPEED_RPM,
                                                      PENDULUM_GEO_MAX_CURRENT_MA))
            {
                PendulumGeometryCalibration_Fail(PENDULUM_GEO_FAULT_PARAMETER_COMMAND, now_ms);
            }
            else
            {
                PendulumGeometryCalibration_Enter(PENDULUM_GEO_PARAM_GAP, now_ms);
            }
        }
        break;

    case PENDULUM_GEO_PARAM_GAP:
        if (PendulumGeometryCalibration_Expired(now_ms, PENDULUM_GEO_COMMAND_GAP_MS))
        {
            if (!BallActuator_SetAbsoluteAngleDeg(PENDULUM_GEO_FIRST_ANGLE_DEG))
            {
                PendulumGeometryCalibration_Fail(PENDULUM_GEO_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                PendulumGeometryCalibration_Enter(PENDULUM_GEO_ZERO_SETTLE, now_ms);
            }
        }
        break;

    case PENDULUM_GEO_ZERO_SETTLE:
        if (PendulumGeometryCalibration_Expired(now_ms, PENDULUM_GEO_ZERO_SETTLE_MS))
        {
            PendulumGeometryCalibration_Enter(PENDULUM_GEO_WAIT_POINT, now_ms);
        }
        break;

    case PENDULUM_GEO_WAIT_POINT:
    case PENDULUM_GEO_LIMIT:
        /* 摆臂保持在最后指令位置。仅由 PA31 推进。 */
        break;

    default:
        PendulumGeometryCalibration_Fail(PENDULUM_GEO_FAULT_POSITION_COMMAND, now_ms);
        break;
    }
}

void PendulumGeometryCalibration_GetStatus(PendulumGeometryCalibrationStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

const char *PendulumGeometryCalibration_StateName(PendulumGeometryCalibrationState state)
{
    switch (state)
    {
    case PENDULUM_GEO_IDLE:
        return "IDLE";
    case PENDULUM_GEO_PRECHECK:
        return "CHECK";
    case PENDULUM_GEO_ENABLE_SETTLE:
        return "ENABLE";
    case PENDULUM_GEO_ZERO_GAP:
        return "ZERO";
    case PENDULUM_GEO_PARAM_GAP:
        return "PARAM";
    case PENDULUM_GEO_ZERO_SETTLE:
        return "SETTLE";
    case PENDULUM_GEO_WAIT_POINT:
        return "WAIT";
    case PENDULUM_GEO_LIMIT:
        return "LIMIT";
    case PENDULUM_GEO_FAULT:
        return "FAULT";
    case PENDULUM_GEO_ABORTED:
        return "ABORT";
    default:
        return "UNKNOWN";
    }
}

#include "pendulum_calibration.h"

#include <string.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/ball_actuator/pendulum_config.h"

static PendulumCalibrationStatus s_status;
static uint32_t s_test_started_ms;
static bool s_fault_stop_sent;
static bool s_result_log_pending;

static void PendulumCalibration_Enter(PendulumCalibrationState state, uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
    if ((state == PENDULUM_CAL_COMPLETE) || (state == PENDULUM_CAL_FAULT) ||
        (state == PENDULUM_CAL_ABORTED))
    {
        s_result_log_pending = true;
    }
}

static void PendulumCalibration_Fail(PendulumCalibrationFault fault, uint32_t now_ms)
{
    s_status.fault = fault;
    s_status.target_angle_deg = 0.0f;
    if (!s_fault_stop_sent)
    {
        (void)BallActuator_Stop();
        (void)BallActuator_Enable(false);
        s_fault_stop_sent = true;
    }
    BallActuator_SetCommandSession(false);
    PendulumCalibration_Enter(PENDULUM_CAL_FAULT, now_ms);
}

static bool PendulumCalibration_Expired(uint32_t now_ms, uint32_t duration_ms)
{
    return (now_ms - s_status.state_started_ms) >= duration_ms;
}

void PendulumCalibration_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = PENDULUM_CAL_IDLE;
    s_result_log_pending = false;
}

bool PendulumCalibration_Start(uint32_t now_ms)
{
    if (PendulumCalibration_IsActive())
    {
        return false;
    }

    ++s_status.sequence;
    s_status.fault = PENDULUM_CAL_FAULT_NONE;
    s_status.target_angle_deg = 0.0f;
    s_test_started_ms = now_ms;
    s_fault_stop_sent = false;
    s_result_log_pending = false;
    BallActuator_SetCommandSession(true);
    PendulumCalibration_Enter(PENDULUM_CAL_PRECHECK, now_ms);
    return true;
}

void PendulumCalibration_Abort(uint32_t now_ms)
{
    if (PendulumCalibration_IsActive())
    {
        (void)BallActuator_Stop();
        (void)BallActuator_Enable(false);
        BallActuator_SetCommandSession(false);
        s_status.target_angle_deg = 0.0f;
        PendulumCalibration_Enter(PENDULUM_CAL_ABORTED, now_ms);
    }
}

bool PendulumCalibration_IsActive(void)
{
    return (s_status.state >= PENDULUM_CAL_PRECHECK) && (s_status.state <= PENDULUM_CAL_DISABLE);
}

bool PendulumCalibration_HasResult(void)
{
    return (s_status.state == PENDULUM_CAL_COMPLETE) || (s_status.state == PENDULUM_CAL_FAULT) ||
           (s_status.state == PENDULUM_CAL_ABORTED);
}

bool PendulumCalibration_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void PendulumCalibration_Task(uint32_t now_ms)
{
    BallActuatorStatus actuator;

    if (!PendulumCalibration_IsActive())
    {
        return;
    }

    if ((now_ms - s_test_started_ms) > PENDULUM_CAL_TOTAL_TIMEOUT_MS)
    {
        PendulumCalibration_Fail(PENDULUM_CAL_FAULT_TOTAL_TIMEOUT, now_ms);
        return;
    }

    BallActuator_GetStatus(&actuator);
    switch (s_status.state)
    {
    case PENDULUM_CAL_PRECHECK:
        if (!actuator.online || !actuator.firmware_x || !actuator.foc_closed_loop)
        {
            PendulumCalibration_Fail(PENDULUM_CAL_FAULT_ACTUATOR_OFFLINE, now_ms);
        }
        else if (!BallActuator_Enable(true))
        {
            PendulumCalibration_Fail(PENDULUM_CAL_FAULT_ENABLE_COMMAND, now_ms);
        }
        else
        {
            PendulumCalibration_Enter(PENDULUM_CAL_ENABLE_SETTLE, now_ms);
        }
        break;

    case PENDULUM_CAL_ENABLE_SETTLE:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_ENABLE_SETTLE_MS))
        {
            if (!BallActuator_ZeroAtCurrentPosition())
            {
                PendulumCalibration_Fail(PENDULUM_CAL_FAULT_ZERO_COMMAND, now_ms);
            }
            else
            {
                PendulumCalibration_Enter(PENDULUM_CAL_ZERO_GAP, now_ms);
            }
        }
        break;

    case PENDULUM_CAL_ZERO_GAP:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_COMMAND_GAP_MS))
        {
            if (!BallActuator_PrepareAbsolutePosition(PENDULUM_CAL_ACCEL_RPM_S,
                                                      PENDULUM_CAL_DECEL_RPM_S,
                                                      PENDULUM_CAL_MAX_SPEED_RPM,
                                                      PENDULUM_CAL_MAX_CURRENT_MA))
            {
                PendulumCalibration_Fail(PENDULUM_CAL_FAULT_PARAMETER_COMMAND, now_ms);
            }
            else
            {
                PendulumCalibration_Enter(PENDULUM_CAL_PARAM_GAP, now_ms);
            }
        }
        break;

    case PENDULUM_CAL_PARAM_GAP:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_COMMAND_GAP_MS))
        {
            s_status.target_angle_deg = PENDULUM_CAL_TEST_ANGLE_DEG;
            if (!BallActuator_SetAbsoluteAngleDeg(s_status.target_angle_deg))
            {
                PendulumCalibration_Fail(PENDULUM_CAL_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                PendulumCalibration_Enter(PENDULUM_CAL_POS_HOLD, now_ms);
            }
        }
        break;

    case PENDULUM_CAL_POS_HOLD:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_POSITION_HOLD_MS))
        {
            s_status.target_angle_deg = 0.0f;
            if (!BallActuator_SetAbsoluteAngleDeg(0.0f))
            {
                PendulumCalibration_Fail(PENDULUM_CAL_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                PendulumCalibration_Enter(PENDULUM_CAL_ZERO1_HOLD, now_ms);
            }
        }
        break;

    case PENDULUM_CAL_ZERO1_HOLD:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_POSITION_HOLD_MS))
        {
            s_status.target_angle_deg = -PENDULUM_CAL_TEST_ANGLE_DEG;
            if (!BallActuator_SetAbsoluteAngleDeg(s_status.target_angle_deg))
            {
                PendulumCalibration_Fail(PENDULUM_CAL_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                PendulumCalibration_Enter(PENDULUM_CAL_NEG_HOLD, now_ms);
            }
        }
        break;

    case PENDULUM_CAL_NEG_HOLD:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_POSITION_HOLD_MS))
        {
            s_status.target_angle_deg = 0.0f;
            if (!BallActuator_SetAbsoluteAngleDeg(0.0f))
            {
                PendulumCalibration_Fail(PENDULUM_CAL_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                PendulumCalibration_Enter(PENDULUM_CAL_ZERO2_HOLD, now_ms);
            }
        }
        break;

    case PENDULUM_CAL_ZERO2_HOLD:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_POSITION_HOLD_MS))
        {
            if (!BallActuator_Stop())
            {
                PendulumCalibration_Fail(PENDULUM_CAL_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                PendulumCalibration_Enter(PENDULUM_CAL_STOP_GAP, now_ms);
            }
        }
        break;

    case PENDULUM_CAL_STOP_GAP:
        if (PendulumCalibration_Expired(now_ms, PENDULUM_CAL_COMMAND_GAP_MS))
        {
            PendulumCalibration_Enter(PENDULUM_CAL_DISABLE, now_ms);
        }
        break;

    case PENDULUM_CAL_DISABLE:
        if (!BallActuator_Enable(false))
        {
            PendulumCalibration_Fail(PENDULUM_CAL_FAULT_ENABLE_COMMAND, now_ms);
        }
        else
        {
            s_status.target_angle_deg = 0.0f;
            BallActuator_SetCommandSession(false);
            PendulumCalibration_Enter(PENDULUM_CAL_COMPLETE, now_ms);
        }
        break;

    default:
        PendulumCalibration_Fail(PENDULUM_CAL_FAULT_TOTAL_TIMEOUT, now_ms);
        break;
    }
}

void PendulumCalibration_GetStatus(PendulumCalibrationStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

const char *PendulumCalibration_StateName(PendulumCalibrationState state)
{
    switch (state)
    {
    case PENDULUM_CAL_IDLE:
        return "IDLE";
    case PENDULUM_CAL_PRECHECK:
        return "CHECK";
    case PENDULUM_CAL_ENABLE_SETTLE:
        return "ENABLE";
    case PENDULUM_CAL_ZERO_GAP:
        return "ZERO";
    case PENDULUM_CAL_PARAM_GAP:
        return "PARAM";
    case PENDULUM_CAL_POS_HOLD:
        return "POS";
    case PENDULUM_CAL_ZERO1_HOLD:
        return "ZERO1";
    case PENDULUM_CAL_NEG_HOLD:
        return "NEG";
    case PENDULUM_CAL_ZERO2_HOLD:
        return "ZERO2";
    case PENDULUM_CAL_STOP_GAP:
        return "STOP";
    case PENDULUM_CAL_DISABLE:
        return "DISABLE";
    case PENDULUM_CAL_COMPLETE:
        return "DONE";
    case PENDULUM_CAL_FAULT:
        return "FAULT";
    case PENDULUM_CAL_ABORTED:
        return "ABORT";
    default:
        return "UNKNOWN";
    }
}

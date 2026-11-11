#include "ball_pulse_id.h"

#include <string.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/ball_pulse_id_config.h"
#include "code/vision_link.h"

static BallPulseIdStatus s_status;
static bool s_result_log_pending;

static float BallPulseId_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool BallPulseId_Expired(uint32_t now_ms, uint32_t interval_ms)
{
    return (now_ms - s_status.state_started_ms) >= interval_ms;
}

static void BallPulseId_Enter(BallPulseIdState state, uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
    if ((state == BALL_PULSE_WAIT_TRIGGER) || (state == BALL_PULSE_DONE) ||
        (state == BALL_PULSE_FAULT) || (state == BALL_PULSE_ABORTED))
    {
        s_result_log_pending = true;
    }
}

static bool BallPulseId_ReadVision(uint32_t now_ms, VisionLinkSample *sample)
{
    if ((!VisionLink_GetLatest(sample)) || (!sample->valid))
    {
        s_status.vision_fresh = false;
        s_status.vision_age_ms = 0xFFFFFFFFUL;
        return false;
    }

    s_status.vision_age_ms = now_ms - sample->rx_time_ms;
    s_status.vision_fresh = s_status.vision_age_ms <= BALL_PULSE_VISION_TIMEOUT_MS;
    if (s_status.vision_fresh)
    {
        s_status.vision_sequence = sample->sequence;
        if (BallPulseId_Abs(sample->position_cm) > BallPulseId_Abs(s_status.peak_cm))
        {
            s_status.peak_cm = sample->position_cm;
        }
        if (BallPulseId_Abs(sample->velocity_cm_s) > BallPulseId_Abs(s_status.peak_velocity_cm_s))
        {
            s_status.peak_velocity_cm_s = sample->velocity_cm_s;
        }
    }
    return s_status.vision_fresh;
}

static void BallPulseId_Fail(BallPulseIdFault fault, uint32_t now_ms)
{
    s_status.fault = fault;
    s_status.target_angle_deg = 0.0f;
    (void)BallActuator_Stop();
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    BallPulseId_Enter(BALL_PULSE_FAULT, now_ms);
}

static void BallPulseId_BeginSafetyReturn(uint32_t now_ms)
{
    s_status.safety_return = true;
    s_status.target_angle_deg = 0.0f;
    if (!BallActuator_SetAbsoluteAngleDeg(0.0f))
    {
        BallPulseId_Fail(BALL_PULSE_FAULT_POSITION_COMMAND, now_ms);
    }
    else
    {
        BallPulseId_Enter(BALL_PULSE_SAFETY_RETURN, now_ms);
    }
}

void BallPulseId_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = BALL_PULSE_IDLE;
}

bool BallPulseId_Start(uint32_t now_ms)
{
    if (BallPulseId_IsActive())
        return false;

    memset(&s_status, 0, sizeof(s_status));
    s_status.state = BALL_PULSE_IDLE;
    s_status.fault = BALL_PULSE_FAULT_NONE;
    s_result_log_pending = false;
    BallPulseId_Enter(BALL_PULSE_PRECHECK, now_ms);
    return true;
}

bool BallPulseId_Trigger(uint32_t now_ms)
{
    VisionLinkSample sample;

    if (s_status.state != BALL_PULSE_WAIT_TRIGGER)
        return false;
    if ((!BallPulseId_ReadVision(now_ms, &sample)) || (!sample.stable))
    {
        return false;
    }
    if (BallPulseId_Abs(sample.position_cm) > BALL_PULSE_START_TOLERANCE_CM)
    {
        s_status.fault = BALL_PULSE_FAULT_START_NOT_CENTERED;
        s_result_log_pending = true;
        return false;
    }

    s_status.start_cm = sample.position_cm;
    s_status.peak_cm = sample.position_cm;
    s_status.end_cm = sample.position_cm;
    s_status.peak_velocity_cm_s = sample.velocity_cm_s;
    s_status.target_angle_deg = BALL_PULSE_TARGET_ANGLE_DEG;
    s_status.safety_return = false;
    if (!BallActuator_SetAbsoluteAngleDeg(s_status.target_angle_deg))
    {
        BallPulseId_Fail(BALL_PULSE_FAULT_POSITION_COMMAND, now_ms);
        return false;
    }
    BallPulseId_Enter(BALL_PULSE_RISE, now_ms);
    return true;
}

void BallPulseId_Abort(uint32_t now_ms)
{
    if (BallPulseId_IsActive())
    {
        (void)BallActuator_Stop();
        (void)BallActuator_Enable(false);
        BallActuator_SetCommandSession(false);
        BallPulseId_Enter(BALL_PULSE_ABORTED, now_ms);
    }
}

bool BallPulseId_IsActive(void)
{
    return (s_status.state >= BALL_PULSE_PRECHECK) && (s_status.state <= BALL_PULSE_SAFETY_RETURN);
}

bool BallPulseId_IsWaitingForTrigger(void)
{
    return s_status.state == BALL_PULSE_WAIT_TRIGGER;
}

bool BallPulseId_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void BallPulseId_Task(uint32_t now_ms)
{
    BallActuatorStatus actuator;
    VisionLinkSample sample;
    bool fresh_vision;

    if (!BallPulseId_IsActive())
        return;

    fresh_vision = BallPulseId_ReadVision(now_ms, &sample);
    BallActuator_GetStatus(&actuator);

    switch (s_status.state)
    {
    case BALL_PULSE_PRECHECK:
        if (!actuator.online || !actuator.firmware_x || !actuator.foc_closed_loop)
        {
            BallPulseId_Fail(BALL_PULSE_FAULT_ACTUATOR_NOT_READY, now_ms);
        }
        else if (!fresh_vision)
        {
            BallPulseId_Fail(BALL_PULSE_FAULT_VISION_STALE, now_ms);
        }
        else
        {
            BallActuator_SetCommandSession(true);
            if (!BallActuator_Enable(true))
            {
                BallPulseId_Fail(BALL_PULSE_FAULT_ENABLE_COMMAND, now_ms);
            }
            else
            {
                BallPulseId_Enter(BALL_PULSE_ENABLE_SETTLE, now_ms);
            }
        }
        break;

    case BALL_PULSE_ENABLE_SETTLE:
        if (!fresh_vision)
        {
            BallPulseId_Fail(BALL_PULSE_FAULT_VISION_STALE, now_ms);
        }
        else if (BallPulseId_Expired(now_ms, BALL_PULSE_ENABLE_SETTLE_MS))
        {
            if (!BallActuator_ZeroAtCurrentPosition())
            {
                BallPulseId_Fail(BALL_PULSE_FAULT_ZERO_COMMAND, now_ms);
            }
            else
            {
                BallPulseId_Enter(BALL_PULSE_ZERO_GAP, now_ms);
            }
        }
        break;

    case BALL_PULSE_ZERO_GAP:
        if (!fresh_vision)
        {
            BallPulseId_Fail(BALL_PULSE_FAULT_VISION_STALE, now_ms);
        }
        else if (BallPulseId_Expired(now_ms, BALL_PULSE_COMMAND_GAP_MS))
        {
            if (!BallActuator_PrepareAbsolutePosition(BALL_PULSE_ACCEL_RPM_S,
                                                      BALL_PULSE_DECEL_RPM_S,
                                                      BALL_PULSE_MAX_SPEED_RPM,
                                                      BALL_PULSE_MAX_CURRENT_MA))
            {
                BallPulseId_Fail(BALL_PULSE_FAULT_PARAMETER_COMMAND, now_ms);
            }
            else
            {
                BallPulseId_Enter(BALL_PULSE_PARAM_GAP, now_ms);
            }
        }
        break;

    case BALL_PULSE_PARAM_GAP:
        if (!fresh_vision)
        {
            BallPulseId_Fail(BALL_PULSE_FAULT_VISION_STALE, now_ms);
        }
        else if (BallPulseId_Expired(now_ms, BALL_PULSE_COMMAND_GAP_MS))
        {
            if (!BallActuator_SetAbsoluteAngleDeg(0.0f))
            {
                BallPulseId_Fail(BALL_PULSE_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                BallPulseId_Enter(BALL_PULSE_ZERO_SETTLE, now_ms);
            }
        }
        break;

    case BALL_PULSE_ZERO_SETTLE:
        if (!fresh_vision)
        {
            BallPulseId_Fail(BALL_PULSE_FAULT_VISION_STALE, now_ms);
        }
        else if (BallPulseId_Expired(now_ms, BALL_PULSE_ZERO_SETTLE_MS))
        {
            BallPulseId_Enter(BALL_PULSE_WAIT_TRIGGER, now_ms);
        }
        break;

    case BALL_PULSE_WAIT_TRIGGER:
        break;

    case BALL_PULSE_RISE:
    case BALL_PULSE_HOLD:
    case BALL_PULSE_RETURN:
    case BALL_PULSE_RESULT_SETTLE:
        if (!fresh_vision)
        {
            BallPulseId_Fail(BALL_PULSE_FAULT_VISION_STALE, now_ms);
        }
        else if (BallPulseId_Abs(sample.position_cm) >= BALL_PULSE_SAFETY_POSITION_CM)
        {
            BallPulseId_BeginSafetyReturn(now_ms);
        }
        else if (s_status.state == BALL_PULSE_RISE &&
                 BallPulseId_Expired(now_ms, BALL_PULSE_RISE_ESTIMATE_MS))
        {
            BallPulseId_Enter(BALL_PULSE_HOLD, now_ms);
        }
        else if (s_status.state == BALL_PULSE_HOLD &&
                 BallPulseId_Expired(now_ms, BALL_PULSE_HOLD_MS))
        {
            s_status.target_angle_deg = 0.0f;
            if (!BallActuator_SetAbsoluteAngleDeg(0.0f))
            {
                BallPulseId_Fail(BALL_PULSE_FAULT_POSITION_COMMAND, now_ms);
            }
            else
            {
                BallPulseId_Enter(BALL_PULSE_RETURN, now_ms);
            }
        }
        else if (s_status.state == BALL_PULSE_RETURN &&
                 BallPulseId_Expired(now_ms, BALL_PULSE_RETURN_ESTIMATE_MS))
        {
            BallPulseId_Enter(BALL_PULSE_RESULT_SETTLE, now_ms);
        }
        else if (s_status.state == BALL_PULSE_RESULT_SETTLE &&
                 BallPulseId_Expired(now_ms, BALL_PULSE_RESULT_SETTLE_MS))
        {
            s_status.end_cm = sample.position_cm;
            BallActuator_SetCommandSession(false);
            BallPulseId_Enter(BALL_PULSE_DONE, now_ms);
        }
        break;

    case BALL_PULSE_SAFETY_RETURN:
        if (BallPulseId_Expired(now_ms, BALL_PULSE_RETURN_ESTIMATE_MS))
        {
            BallActuator_SetCommandSession(false);
            BallPulseId_Enter(BALL_PULSE_DONE, now_ms);
        }
        break;

    default:
        BallPulseId_Fail(BALL_PULSE_FAULT_POSITION_COMMAND, now_ms);
        break;
    }
}

void BallPulseId_GetStatus(BallPulseIdStatus *status)
{
    if (status != NULL)
        *status = s_status;
}

const char *BallPulseId_StateName(BallPulseIdState state)
{
    switch (state)
    {
    case BALL_PULSE_IDLE:
        return "IDLE";
    case BALL_PULSE_PRECHECK:
        return "CHECK";
    case BALL_PULSE_ENABLE_SETTLE:
        return "ENABLE";
    case BALL_PULSE_ZERO_GAP:
        return "ZERO";
    case BALL_PULSE_PARAM_GAP:
        return "PARAM";
    case BALL_PULSE_ZERO_SETTLE:
        return "SETTLE";
    case BALL_PULSE_WAIT_TRIGGER:
        return "WAIT";
    case BALL_PULSE_RISE:
        return "RISE";
    case BALL_PULSE_HOLD:
        return "HOLD";
    case BALL_PULSE_RETURN:
        return "RETURN";
    case BALL_PULSE_RESULT_SETTLE:
        return "RESULT";
    case BALL_PULSE_SAFETY_RETURN:
        return "SAFE";
    case BALL_PULSE_DONE:
        return "DONE";
    case BALL_PULSE_FAULT:
        return "FAULT";
    case BALL_PULSE_ABORTED:
        return "ABORT";
    default:
        return "UNKNOWN";
    }
}

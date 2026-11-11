#include "app_telemetry.h"

#include "app/ball_actuator/pendulum_calibration.h"
#include "app/q3/ball_mechanism_id.h"
#include "app/q3/ball_position_control.h"
#include "app/q3/ball_pulse_id.h"
#include "app/q3/pendulum_geometry_calibration.h"
#include "app/q3/q3_sequence.h"
#include "app/q4/q45_vehicle_control.h"
#include "app/q6/q6_config.h"
#include "code/debug_log.h"

static uint32_t s_vision_log_last_ms;
static uint32_t s_control_log_last_ms;
static uint32_t s_actuator_log_last_ms;
static uint32_t s_vision_log_period_ms = 500U;
static uint32_t s_control_log_period_ms = 200U;
static uint32_t s_actuator_log_period_ms = 1000U;

void AppTelemetry_BeginCycle(void)
{
    s_vision_log_period_ms = 500U;
    s_control_log_period_ms = 200U;
    s_actuator_log_period_ms = 1000U;
#if Q6_TASK_ENABLE
    {
        Q45VehicleStatus vehicle;

        Q45Vehicle_GetStatus(&vehicle);
        if ((vehicle.mode == Q45_VEHICLE_MODE_Q6) && (vehicle.state == Q45_VEHICLE_RUNNING))
        {
            s_vision_log_period_ms = 200U;
            s_control_log_period_ms = 100U;
            s_actuator_log_period_ms = 500U;
        }
    }
#endif
}

void AppTelemetry_PublishDue(uint32_t now_ms)
{
    if ((now_ms - s_vision_log_last_ms) >= s_vision_log_period_ms)
    {
        s_vision_log_last_ms = now_ms;
        DebugLog_PublishVision(now_ms);
    }
    if ((now_ms - s_control_log_last_ms) >= s_control_log_period_ms)
    {
        s_control_log_last_ms = now_ms;
        if (PendulumCalibration_IsActive() || PendulumCalibration_ConsumeResultLog())
        {
            DebugLog_PublishPendulumCalibration(now_ms);
        }
        if (BallPositionControl_IsActive() || BallPositionControl_ConsumeResultLog())
        {
            DebugLog_PublishBallPositionControl(now_ms);
        }
        if (Q3Sequence_IsActive() || Q3Sequence_ConsumeResultLog())
        {
            DebugLog_PublishQ3Sequence(now_ms);
        }
        if (Q45Vehicle_IsActive() || Q45Vehicle_ConsumeResultLog())
        {
            DebugLog_PublishQ45Vehicle(now_ms);
        }
        if (BallMechanismId_IsActive() || BallMechanismId_ConsumeResultLog())
        {
            DebugLog_PublishBallMechanismId(now_ms);
        }
        if (PendulumGeometryCalibration_IsActive() ||
            PendulumGeometryCalibration_ConsumeResultLog())
        {
            DebugLog_PublishPendulumGeometryCalibration(now_ms);
        }
        if (BallPulseId_IsActive() || BallPulseId_ConsumeResultLog())
        {
            DebugLog_PublishBallPulseId(now_ms);
        }
        if ((now_ms - s_actuator_log_last_ms) >= s_actuator_log_period_ms)
        {
            s_actuator_log_last_ms = now_ms;
            DebugLog_PublishActuator(now_ms);
        }
    }
}

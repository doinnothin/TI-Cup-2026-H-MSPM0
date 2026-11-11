#include <stdbool.h>
#include <stdint.h>

#include "app/app_telemetry.h"
#include "app/q4/q45_vehicle_control.h"

static unsigned checks;
static unsigned failures;
static unsigned first_failure_line;
unsigned failure_lines[32];
unsigned failure_lines_count;

static Q45VehicleStatus vehicle_status;
static unsigned vision_published;
static unsigned actuator_published;
static unsigned control_records_published;

#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        ++checks;                                                                                  \
        if (!(condition))                                                                          \
        {                                                                                          \
            if (failure_lines_count < 32U)                                                         \
            {                                                                                      \
                failure_lines[failure_lines_count++] = __LINE__;                                   \
            }                                                                                      \
            ++failures;                                                                            \
            if (first_failure_line == 0U)                                                          \
            {                                                                                      \
                first_failure_line = __LINE__;                                                     \
            }                                                                                      \
        }                                                                                          \
    } while (0)

void Q45Vehicle_GetStatus(Q45VehicleStatus *status)
{
    *status = vehicle_status;
}

bool Q45Vehicle_IsActive(void)
{
    return true;
}
bool Q45Vehicle_ConsumeResultLog(void)
{
    return false;
}
bool PendulumCalibration_IsActive(void)
{
    return true;
}
bool PendulumCalibration_ConsumeResultLog(void)
{
    return false;
}
bool BallPositionControl_IsActive(void)
{
    return true;
}
bool BallPositionControl_ConsumeResultLog(void)
{
    return false;
}
bool Q3Sequence_IsActive(void)
{
    return true;
}
bool Q3Sequence_ConsumeResultLog(void)
{
    return false;
}
bool BallMechanismId_IsActive(void)
{
    return true;
}
bool BallMechanismId_ConsumeResultLog(void)
{
    return false;
}
bool PendulumGeometryCalibration_IsActive(void)
{
    return true;
}
bool PendulumGeometryCalibration_ConsumeResultLog(void)
{
    return false;
}
bool BallPulseId_IsActive(void)
{
    return true;
}
bool BallPulseId_ConsumeResultLog(void)
{
    return false;
}

void DebugLog_PublishVision(uint32_t now_ms)
{
    (void)now_ms;
    ++vision_published;
}

void DebugLog_PublishActuator(uint32_t now_ms)
{
    (void)now_ms;
    ++actuator_published;
}

void DebugLog_PublishPendulumCalibration(uint32_t now_ms)
{
    (void)now_ms;
    ++control_records_published;
}

void DebugLog_PublishBallPositionControl(uint32_t now_ms)
{
    (void)now_ms;
    ++control_records_published;
}

void DebugLog_PublishQ3Sequence(uint32_t now_ms)
{
    (void)now_ms;
    ++control_records_published;
}

void DebugLog_PublishQ45Vehicle(uint32_t now_ms)
{
    (void)now_ms;
    ++control_records_published;
}

void DebugLog_PublishBallMechanismId(uint32_t now_ms)
{
    (void)now_ms;
    ++control_records_published;
}

void DebugLog_PublishPendulumGeometryCalibration(uint32_t now_ms)
{
    (void)now_ms;
    ++control_records_published;
}

void DebugLog_PublishBallPulseId(uint32_t now_ms)
{
    (void)now_ms;
    ++control_records_published;
}

static void PublishAt(uint32_t now_ms)
{
    AppTelemetry_BeginCycle();
    AppTelemetry_PublishDue(now_ms);
}

int main(void)
{
    vehicle_status.mode = Q45_VEHICLE_MODE_Q4;
    vehicle_status.state = Q45_VEHICLE_READY;
    PublishAt(199U);
    CHECK(vision_published == 0U && control_records_published == 0U);

    PublishAt(200U);
    CHECK(control_records_published == 7U);
    CHECK(vision_published == 0U && actuator_published == 0U);

    PublishAt(500U);
    CHECK(vision_published == 1U);
    CHECK(control_records_published == 14U);
    PublishAt(1000U);
    CHECK(vision_published == 2U);
    CHECK(actuator_published == 1U);

    vehicle_status.mode = Q45_VEHICLE_MODE_Q6;
    vehicle_status.state = Q45_VEHICLE_RUNNING;
    PublishAt(1100U);
    CHECK(control_records_published == 28U);
    PublishAt(1200U);
    CHECK(control_records_published == 35U);
    CHECK(vision_published == 3U);
    PublishAt(1500U);
    CHECK(vision_published == 4U);
    CHECK(actuator_published == 2U);

    vehicle_status.state = Q45_VEHICLE_READY;
    PublishAt(1600U);
    CHECK(vision_published == 4U);
    PublishAt(2000U);
    CHECK(vision_published == 5U);

    return failures == 0U ? 0 : 1;
}

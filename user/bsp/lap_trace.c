#include "lap_trace.h"

#include "licheng.h"

static volatile uint32_t s_elapsed_ms;
static volatile LapTraceState_t s_state = LAP_TRACE_IDLE;
static volatile uint8_t s_stop_requested;
static volatile float s_stop_distance_cm = LAP_TRACE_STOP_DISTANCE_CM;
static volatile uint32_t s_time_limit_ms = LAP_TRACE_TIME_LIMIT_MS;

void LapTrace_Reset(void)
{
    s_elapsed_ms = 0U;
    s_state = LAP_TRACE_IDLE;
    s_stop_requested = 0U;
    s_stop_distance_cm = LAP_TRACE_STOP_DISTANCE_CM;
    s_time_limit_ms = LAP_TRACE_TIME_LIMIT_MS;
}

void LapTrace_Start(void)
{
    LapTrace_StartToDistance(LAP_TRACE_STOP_DISTANCE_CM, LAP_TRACE_TIME_LIMIT_MS);
}

void LapTrace_StartToDistance(float stop_distance_cm, uint32_t time_limit_ms)
{
    s_elapsed_ms = 0U;
    s_stop_requested = 0U;
    s_stop_distance_cm = (stop_distance_cm > 0.0f) ? stop_distance_cm : LAP_TRACE_STOP_DISTANCE_CM;
    s_time_limit_ms = (time_limit_ms != 0U) ? time_limit_ms : LAP_TRACE_TIME_LIMIT_MS;
    s_state = LAP_TRACE_RUNNING;
}

void LapTrace_Tick10ms(void)
{
    if (s_state != LAP_TRACE_RUNNING)
    {
        return;
    }

    s_elapsed_ms += 10U;
    if (s_elapsed_ms >= s_time_limit_ms)
    {
        s_state = LAP_TRACE_TIMEOUT;
        s_stop_requested = 1U;
        return;
    }

    if (g_licheng.center_cm >= s_stop_distance_cm)
    {
        s_state = LAP_TRACE_FINISHED;
        s_stop_requested = 1U;
    }
}

uint32_t LapTrace_GetElapsedMs(void)
{
    return s_elapsed_ms;
}

LapTraceState_t LapTrace_GetState(void)
{
    return s_state;
}

uint8_t LapTrace_StopRequested(void)
{
    return s_stop_requested;
}

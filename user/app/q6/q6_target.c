#include "q6_target.h"

#include <math.h>
#include <string.h>

#include "q6_config.h"

#define Q6_TARGET_HISTORY_CAPACITY 32U

#if Q6_STATIC_TEST_MODE
#define Q6_CAPTURE_INPUT_LIMIT_CM (Q6_TEST_TARGET_LIMIT_CM + Q6_EXPLICIT_START_TOLERANCE_CM)
#define Q6_CAPTURE_LOCK_LIMIT_CM Q6_TEST_TARGET_LIMIT_CM
#else
#define Q6_CAPTURE_INPUT_LIMIT_CM Q6_DYNAMIC_CAPTURE_MAX_POSITION_CM
#define Q6_CAPTURE_LOCK_LIMIT_CM Q6_DYNAMIC_CAPTURE_MAX_POSITION_CM
#endif

static Q6TargetStatus s_status;
static float s_positions[Q6_TARGET_HISTORY_CAPACITY];
static uint32_t s_rx_times[Q6_TARGET_HISTORY_CAPACITY];
static uint8_t s_next;
static uint8_t s_count;
static uint32_t s_previous_rx_ms;
static uint16_t s_previous_source_ms;
static uint8_t s_previous_sequence;
static bool s_have_previous;

static float Q6Target_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

static void Q6Target_ClearWindow(void)
{
    s_count = 0U;
    s_next = 0U;
    s_have_previous = false;
    s_status.ready = false;
    s_status.independent_frames = 0U;
    s_status.span_ms = 0U;
}

void Q6Target_Init(Q6TargetSource source, float explicit_target_cm)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.source = source;
    s_status.requested_cm = explicit_target_cm;
    Q6Target_ClearWindow();
}

bool Q6Target_IsPixelRowAccepted(uint16_t pixel_cy)
{
    uint32_t row_offset;

    if (Q6_BALL_PIXEL_CY_BAND == 0U)
    {
        return true;
    }
    row_offset = (pixel_cy > Q6_BALL_PIXEL_CY_CENTER)
                     ? ((uint32_t)pixel_cy - Q6_BALL_PIXEL_CY_CENTER)
                     : ((uint32_t)Q6_BALL_PIXEL_CY_CENTER - pixel_cy);
    return row_offset <= Q6_BALL_PIXEL_CY_BAND;
}

void Q6Target_Update(const Q6TargetFrame *sample, uint32_t now_ms)
{
    uint8_t oldest;
    uint8_t i;
    float min_cm;
    float max_cm;
    float first_cm;
    uint32_t first_ms;
    float ordered[Q6_TARGET_HISTORY_CAPACITY];

    if (s_status.locked)
        return;
    if (sample == NULL || !sample->valid || !isfinite(sample->position_cm) ||
        !isfinite(sample->velocity_cm_s) ||
        Q6Target_Abs(sample->velocity_cm_s) > Q6_CAPTURE_MAX_SPEED_CM_S ||
        sample->confidence_pct < Q6_CAPTURE_MIN_CONFIDENCE ||
        (int32_t)(now_ms - sample->rx_time_ms) < 0 ||
        (now_ms - sample->rx_time_ms) > Q6_CAPTURE_MAX_AGE_MS ||
        Q6Target_Abs(sample->position_cm) > ((s_status.source == Q6_TARGET_EXPLICIT)
                                                 ? Q6_EXPLICIT_CAPTURE_MAX_POSITION_CM
                                                 : Q6_CAPTURE_INPUT_LIMIT_CM))
    {
        ++s_status.rejected_frames;
        Q6Target_ClearWindow();
        return;
    }
    if (s_have_previous && sample->rx_time_ms == s_previous_rx_ms)
        return;
    if (s_have_previous && ((uint8_t)(sample->sequence - s_previous_sequence) == 0U ||
                            (uint16_t)(sample->source_timestamp_ms - s_previous_source_ms) == 0U))
    {
        /* 带新 UART 到达的重放视觉数据不是新图像。 */
        ++s_status.rejected_frames;
        if (sample->rx_time_ms - s_previous_rx_ms > Q6_CAPTURE_MAX_FRAME_GAP_MS)
            Q6Target_ClearWindow();
        return;
    }
    if (s_have_previous && ((int32_t)(sample->rx_time_ms - s_previous_rx_ms) <= 0 ||
                            sample->rx_time_ms - s_previous_rx_ms > Q6_CAPTURE_MAX_FRAME_GAP_MS ||
                            (uint8_t)(sample->sequence - s_previous_sequence) >= 128U ||
                            (uint16_t)(sample->source_timestamp_ms - s_previous_source_ms) >
                                Q6_CAPTURE_MAX_FRAME_GAP_MS))
    {
        ++s_status.rejected_frames;
        Q6Target_ClearWindow();
    }

    s_previous_rx_ms = sample->rx_time_ms;
    s_previous_source_ms = sample->source_timestamp_ms;
    s_previous_sequence = sample->sequence;
    s_have_previous = true;
    s_positions[s_next] = sample->position_cm;
    s_rx_times[s_next] = sample->rx_time_ms;
    s_next = (uint8_t)((s_next + 1U) % Q6_TARGET_HISTORY_CAPACITY);
    if (s_count < Q6_TARGET_HISTORY_CAPACITY)
        ++s_count;
    oldest =
        (uint8_t)((s_next + Q6_TARGET_HISTORY_CAPACITY - s_count) % Q6_TARGET_HISTORY_CAPACITY);
    first_ms = s_rx_times[oldest];
    first_cm = s_positions[oldest];
    s_status.independent_frames = s_count;
    s_status.span_ms = sample->rx_time_ms - first_ms;

    min_cm = sample->position_cm;
    max_cm = sample->position_cm;
    for (i = 0U; i < s_count; ++i)
    {
        float p = s_positions[(oldest + i) % Q6_TARGET_HISTORY_CAPACITY];
        uint8_t j = i;
        if (p < min_cm)
            min_cm = p;
        if (p > max_cm)
            max_cm = p;
        while (j > 0U && ordered[j - 1U] > p)
        {
            ordered[j] = ordered[j - 1U];
            --j;
        }
        ordered[j] = p;
    }

    /* 在显式模式下，小球静止后即锁定请求值；
     * 随后 RUNNING 运动曲线将其带向该目标，因此此处
     * 启动容差不应限制就绪窗口。 */
    if (max_cm - min_cm > Q6_CAPTURE_MAX_POSITION_RANGE_CM)
    {
        /* 运动中的小球必须重新建立 500 ms 的稳定窗口。 */
        Q6Target_ClearWindow();
        s_previous_rx_ms = sample->rx_time_ms;
        s_previous_source_ms = sample->source_timestamp_ms;
        s_previous_sequence = sample->sequence;
        s_have_previous = true;
        s_positions[0] = sample->position_cm;
        s_rx_times[0] = sample->rx_time_ms;
        s_count = 1U;
        s_next = 1U;
        s_status.independent_frames = 1U;
        return;
    }
    s_status.ready = s_count >= Q6_CAPTURE_MIN_FRAMES &&
                     s_status.span_ms >= Q6_CAPTURE_MIN_SPAN_MS && (s_status.span_ms != 0U) &&
                     ((Q6Target_Abs(sample->position_cm - first_cm) * 1000.0f /
                       (float)s_status.span_ms) <= Q6_CAPTURE_MAX_SPEED_CM_S);
    if (s_status.ready)
    {
        s_status.candidate_cm =
            s_status.source == Q6_TARGET_CAPTURE ? ordered[s_count / 2U] : s_status.requested_cm;
        if (Q6Target_Abs(s_status.candidate_cm) > Q6_CAPTURE_LOCK_LIMIT_CM)
            s_status.ready = false;
    }
}

bool Q6Target_Lock(float *target_cm)
{
    if (!s_status.ready || s_status.locked || target_cm == NULL)
        return false;
    s_status.locked = true;
    s_status.locked_cm = s_status.candidate_cm;
    *target_cm = s_status.locked_cm;
    return true;
}

void Q6Target_GetStatus(Q6TargetStatus *status)
{
    if (status != NULL)
        *status = s_status;
}

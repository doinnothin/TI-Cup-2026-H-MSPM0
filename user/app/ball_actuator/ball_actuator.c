#include "ball_actuator.h"

#include <string.h>

#include "code/x42s.h"

#define BALL_ACTUATOR_DEFAULT_ADDRESS 1U
#define BALL_ACTUATOR_OPTION_PERIOD_MS 500U
#define BALL_ACTUATOR_POSITION_PERIOD_MS 50U
#define BALL_ACTUATOR_REPLY_TIMEOUT_MS 100U
#define BALL_ACTUATOR_ONLINE_TIMEOUT_MS 1500U

typedef enum
{
    BALL_ACTUATOR_REQUEST_NONE = 0,
    BALL_ACTUATOR_REQUEST_OPTIONS,
    BALL_ACTUATOR_REQUEST_POSITION
} BallActuatorRequest;

static BallActuatorStatus s_status;
static uint32_t s_last_request_ms;
static uint32_t s_last_position_request_ms;
static uint32_t s_last_frame_count;
static uint32_t s_last_option_frame_count;
static uint32_t s_last_position_frame_count;
static BallActuatorRequest s_pending_request;

void BallActuator_Init(uint8_t address)
{
    uint8_t selected_address = (address == 0U) ? BALL_ACTUATOR_DEFAULT_ADDRESS : address;

    memset(&s_status, 0, sizeof(s_status));
    s_status.address = selected_address;
    /* 使首次只读请求在第一个 10 ms 调度节拍即发生，
     * 而无需在复位后等待一整个请求周期。 */
    s_last_request_ms = (uint32_t)(0U - BALL_ACTUATOR_OPTION_PERIOD_MS);
    s_last_position_request_ms = (uint32_t)(0U - BALL_ACTUATOR_POSITION_PERIOD_MS);
    s_last_frame_count = 0U;
    s_last_option_frame_count = 0U;
    s_last_position_frame_count = 0U;
    s_pending_request = BALL_ACTUATOR_REQUEST_NONE;

    /* X42S_Init 仅复位接收状态并使能 UART0 RX 中断。 */
    X42S_Init(selected_address);
}

void BallActuator_Task(uint32_t now_ms)
{
    const X42S_State *motor = X42S_GetState();
    bool has_new_frame;
    bool has_new_options;
    bool has_new_position;

    X42S_Process();

    has_new_frame = motor->frames_received != s_last_frame_count;
    if (has_new_frame)
    {
        s_last_frame_count = motor->frames_received;
        s_status.last_reply_ms = now_ms;
        s_status.last_function = motor->last_function;
        s_status.last_reply = motor->last_reply;
        s_status.status_flags = motor->status_flags;
    }

    has_new_options = ((motor->valid_mask & X42S_VALID_OPTIONS) != 0U) &&
                      (motor->option_frames_received != s_last_option_frame_count);
    if (has_new_options)
    {
        s_last_option_frame_count = motor->option_frames_received;
        s_status.option_flags = motor->option_flags;
        s_status.options_valid = true;
        s_status.firmware_x = ((motor->option_flags & X42S_OPTION_FIRMWARE_EMM) == 0U);
        s_status.foc_closed_loop = ((motor->option_flags & X42S_OPTION_CLOSED_LOOP) != 0U);
        s_status.last_option_ms = now_ms;
        if (s_pending_request == BALL_ACTUATOR_REQUEST_OPTIONS)
        {
            s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
        }
    }

    has_new_position = ((motor->valid_mask & X42S_VALID_POSITION) != 0U) &&
                       (motor->position_frames_received != s_last_position_frame_count);
    if (has_new_position)
    {
        s_last_position_frame_count = motor->position_frames_received;
        s_status.actual_angle_deg = motor->position_deg;
        s_status.actual_angle_valid = true;
        s_status.last_position_ms = now_ms;
        if (s_pending_request == BALL_ACTUATOR_REQUEST_POSITION)
        {
            s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
        }
    }

    if ((s_pending_request != BALL_ACTUATOR_REQUEST_NONE) &&
        ((now_ms - s_last_request_ms) >= BALL_ACTUATOR_REPLY_TIMEOUT_MS))
    {
        if (s_pending_request == BALL_ACTUATOR_REQUEST_OPTIONS)
        {
            ++s_status.request_timeouts;
        }
        else
        {
            ++s_status.position_request_timeouts;
        }
        s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
    }

    /* 在位置轮询开始前先检查配置。此后
     * 两类请求共用同一个挂起槽位，因此回复不会被
     * 相互混淆。运动过程中仅轮询位置；这
     * 避免了将缓慢的 option 查询插入控制命令流中。 */
    if (s_pending_request == BALL_ACTUATOR_REQUEST_NONE)
    {
        if ((!s_status.command_session_active) &&
            ((!s_status.options_valid) ||
             ((now_ms - s_status.last_option_ms) >= BALL_ACTUATOR_OPTION_PERIOD_MS)))
        {
            if (X42S_RequestOptions())
            {
                ++s_status.requests_sent;
                s_last_request_ms = now_ms;
                s_pending_request = BALL_ACTUATOR_REQUEST_OPTIONS;
            }
        }
        else if (s_status.options_valid &&
                 ((now_ms - s_last_position_request_ms) >= BALL_ACTUATOR_POSITION_PERIOD_MS))
        {
            if (X42S_RequestFeedback(X42S_FEEDBACK_POSITION))
            {
                ++s_status.position_requests_sent;
                s_last_request_ms = now_ms;
                s_last_position_request_ms = now_ms;
                s_pending_request = BALL_ACTUATOR_REQUEST_POSITION;
            }
        }
    }

    s_status.online = s_status.options_valid &&
                      ((now_ms - s_status.last_reply_ms) <= BALL_ACTUATOR_ONLINE_TIMEOUT_MS);
}

void BallActuator_GetStatus(BallActuatorStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

void BallActuator_SetCommandSession(bool active)
{
    s_status.command_session_active = active;
    if (active)
    {
        /* 在受控序列期间，延迟的 option 回复是无害的；
         * 之后不要将其计为由有意暂停轮询导致的超时。 */
        if (s_pending_request == BALL_ACTUATOR_REQUEST_OPTIONS)
        {
            s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
        }
    }
}

static bool BallActuator_CommandAllowed(void)
{
    /* 在运动会话期间，会有意暂停 option 轮询，使
     * 查询无法与 F1/FC 运动帧交错。`online` 仍会
     * 如实报告新回复看门狗状态，但会话前已建立的 X 固件
     * 身份足以用于发送 F3 命令。 */
    return (s_status.online || s_status.command_session_active) && s_status.options_valid &&
           s_status.firmware_x;
}

static bool BallActuator_MotionAllowed(void)
{
    return BallActuator_CommandAllowed() && s_status.foc_closed_loop;
}

bool BallActuator_Enable(bool enable)
{
    if (!enable)
    {
        /* 即使上次状态已陈旧，也必须尝试故障停机。 */
        return X42S_Enable(false);
    }
    /* F3 是用于恢复被有意禁用的 X42S 的命令。
     * 此处要求已使能的 FOC 位曾使一次已完成的 Q3 运行
     * 在复位后无法再次准备。位置/零位命令仍
     * 保留更严格的 MotionAllowed 闭环保护。 */
    return BallActuator_CommandAllowed() && X42S_Enable(true);
}

bool BallActuator_ClearProtection(void)
{
    /* X42S 手册：0E 52 清除堵转/过流保护。在每次运动尝试
     * 前发送是安全的；不会生成位置命令。 */
    return BallActuator_MotionAllowed() && X42S_ClearProtection();
}

bool BallActuator_Stop(void)
{
    return X42S_Stop();
}

bool BallActuator_ZeroAtCurrentPosition(void)
{
    if (BallActuator_MotionAllowed() && X42S_ZeroCurrentPosition())
    {
        s_status.commanded_angle_deg = 0.0f;
        s_status.commanded_angle_valid = true;
        return true;
    }
    return false;
}

bool BallActuator_ReturnAbsoluteZero(void)
{
    return BallActuator_MotionAllowed() && X42S_ReturnAbsoluteZero();
}

bool BallActuator_PrepareAbsolutePosition(uint16_t acceleration_rpm_s,
                                          uint16_t deceleration_rpm_s,
                                          float max_speed_rpm,
                                          uint16_t max_current_ma)
{
    return BallActuator_MotionAllowed() && X42S_SetQuickPositionParams(acceleration_rpm_s,
                                                                       deceleration_rpm_s,
                                                                       max_speed_rpm,
                                                                       X42S_POSITION_ABSOLUTE,
                                                                       max_current_ma);
}

bool BallActuator_SetAbsoluteAngleDeg(float target_angle_deg)
{
    if (BallActuator_MotionAllowed() && X42S_SetQuickPosition(target_angle_deg))
    {
        s_status.commanded_angle_deg = target_angle_deg;
        s_status.commanded_angle_valid = true;
        return true;
    }
    return false;
}

#ifndef BALL_ACTUATOR_H
#define BALL_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * X42S 摆杆执行器的应用层保护。
 *
 * option 轮询在任何运动前校验 UART、固件与控制模式。
 * 运动调用仍受该已校验状态的保护。
 */

typedef struct
{
    uint8_t address;
    uint16_t option_flags;
    uint8_t last_function;
    uint8_t last_reply;
    uint8_t status_flags;
    uint32_t requests_sent;
    uint32_t request_timeouts;
    uint32_t position_requests_sent;
    uint32_t position_request_timeouts;
    uint32_t last_option_ms;
    uint32_t last_position_ms;
    uint32_t last_reply_ms;
    float commanded_angle_deg;
    float actual_angle_deg;
    bool options_valid;
    bool commanded_angle_valid;
    bool actual_angle_valid;
    bool online;
    bool firmware_x;
    bool foc_closed_loop;
    bool command_session_active;
} BallActuatorStatus;

/* 初始化 X42S 协议驱动。此函数绝不驱动或
 * 使能电机。地址 0 选择文档规定的默认值地址 1。 */
void BallActuator_Init(uint8_t address);

/* 在 10 ms 主循环调度器中调用。它将缓慢的 option
 * 检查与 20 Hz 的实际位置反馈请求串行化。 */
void BallActuator_Task(uint32_t now_ms);

/* 返回已校验的只读执行器状态快照。 */
void BallActuator_GetStatus(BallActuatorStatus *status);

/* 在时间关键的序列下发电机命令期间，暂停后台 0x1A option 轮询。
 * 现有配置仍然有效，且每个收到的 X42S 回复
 * 仍会刷新在线看门狗。 */
void BallActuator_SetCommandSession(bool active);

/* 以下命令受已校验的 X/FOC 配置保护。
 * 在受控会话期间，由于 option 轮询被有意暂停，`online` 可能超时失效；
 * 但先前已校验的配置仍允许
 * 新的运动命令。机械限位仍保留在控制器层。 */
bool BallActuator_Enable(bool enable);
bool BallActuator_ClearProtection(void);
bool BallActuator_Stop(void);
bool BallActuator_ZeroAtCurrentPosition(void);
bool BallActuator_ReturnAbsoluteZero(void);
bool BallActuator_PrepareAbsolutePosition(uint16_t acceleration_rpm_s,
                                          uint16_t deceleration_rpm_s,
                                          float max_speed_rpm,
                                          uint16_t max_current_ma);
bool BallActuator_SetAbsoluteAngleDeg(float target_angle_deg);

#endif

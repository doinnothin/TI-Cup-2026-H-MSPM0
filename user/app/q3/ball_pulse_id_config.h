#ifndef BALL_PULSE_ID_CONFIG_H
#define BALL_PULSE_ID_CONFIG_H

/* 负方向启动实验：0 -> -6.5 -> 0。
 * 负摆杆角度应使球朝视觉正方向移动。 */
#define BALL_PULSE_TARGET_ANGLE_DEG -7.0f
#define BALL_PULSE_MAX_SPEED_RPM 8.0f
#define BALL_PULSE_ACCEL_RPM_S 100U
#define BALL_PULSE_DECEL_RPM_S 100U
#define BALL_PULSE_MAX_CURRENT_MA 600U

/* 脉冲测试仍使用保守的时间边界。实际电机
 * 位置现在已独立记录，并将在反馈链通过硬件验证后，
 * 于正式 Q3 控制器中取代此估计值。 */
#define BALL_PULSE_RISE_ESTIMATE_MS 190U
/* 在估计到达 -7 deg 后立即返回：这是
 * 动态惯量测试，而非重力保持测试。 */
#define BALL_PULSE_HOLD_MS 0U
#define BALL_PULSE_RETURN_ESTIMATE_MS 500U
#define BALL_PULSE_RESULT_SETTLE_MS 500U

#define BALL_PULSE_ENABLE_SETTLE_MS 300U
#define BALL_PULSE_COMMAND_GAP_MS 100U
#define BALL_PULSE_ZERO_SETTLE_MS 500U
#define BALL_PULSE_VISION_TIMEOUT_MS 250U
#define BALL_PULSE_START_TOLERANCE_CM 1.0f
#define BALL_PULSE_SAFETY_POSITION_CM 6.0f

#endif

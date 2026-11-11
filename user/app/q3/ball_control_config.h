#ifndef BALL_CONTROL_CONFIG_H
#define BALL_CONTROL_CONFIG_H

/* Q3 通用外环控制器。Q3Sequence 提供 +5 cm 与 -5 cm 目标。 */
#define BALL_CONTROL_START_WINDOW_CM 1.0f
#define BALL_CONTROL_TARGET_TOLERANCE_CM 1.0f
#define BALL_CONTROL_SAFETY_POSITION_CM 11.0f
#define BALL_CONTROL_TARGET_LIMIT_CM 9.0f
#define BALL_CONTROL_CONFIDENCE_MIN 50U
#define BALL_CONTROL_SETTLE_SPEED_CM_S 0.5f
#define BALL_CONTROL_SAMPLE_MAX_MS 250U
#define BALL_CONTROL_SAMPLE_MAX_S 0.25f
#define BALL_CONTROL_KICK_MOVE_CM 0.2f
/* 0：Q3 中心 -> +5 -> -5 保持；1：任意目标保持。 */
#define BALL_CONTROL_SINGLE_TARGET_MODE 0U
#define BALL_CONTROL_SINGLE_TARGET_CM 5.0f

/* 摄像头状态观测器。速度是估计的阻尼信号，并非
 * 单独测量的状态，因此不再占用另一个 PID 环。 */
#define BALL_ESTIMATOR_ALPHA 0.65f
#define BALL_ESTIMATOR_BETA 0.08f
#define BALL_ESTIMATOR_INNOVATION_LIMIT_CM 2.5f
#define BALL_ESTIMATOR_REACQUIRE_COUNT 3U

/* 球体外环 PD，以物理横梁角度表示。这些是由 0.015 deg/mm 与
 * 0.013 deg/(mm/s) 换算得到的保守初值。
 * Kd/Kp ~= 0.87 s 是用于制动的近似速度前视时间。 */
#define BALL_OUTER_KP_DEG_PER_CM 0.17f
/* 当球仍远离目标时使用适度的阻尼。仅在最终
 * 制动区内恢复完整 D，避免在管体局部静摩擦点附近
 * 过早反向。 */
#define BALL_OUTER_KD_CRUISE_DEG_PER_CM_S 0.04f
#define BALL_OUTER_KD_DEG_PER_CM_S 0.13f
#define BALL_OUTER_RETURN_CRUISE_KD_DEG_PER_CM_S 0.04f
#define BALL_OUTER_RETURN_BRAKE_KD_DEG_PER_CM_S 0.14f
#define BALL_OUTER_RETURN_HOLD_KD_DEG_PER_CM_S 0.08f
#define BALL_OUTER_BRAKE_ZONE_CM 1.2f
#define BALL_OUTER_FEEDFORWARD_DEG 0.0f
#define BALL_OUTER_POSITION_DEADBAND_CM 0.3f
/* 经我们测得的高度表换算，0.9 度横梁角度约对应 7.9 度电机
 * 角度。25 度电机角度仅作为机械应急边界。 */
#define BALL_OUTER_BEAM_LIMIT_DEG 0.9f
#define BALL_OUTER_RETURN_CRUISE_BEAM_LIMIT_DEG 0.75f
#define BALL_OUTER_RETURN_BRAKE_BEAM_LIMIT_DEG 0.45f
#define BALL_OUTER_RETURN_HOLD_BEAM_LIMIT_DEG 0.30f
#define BALL_OUTER_BEAM_RATE_LIMIT_DEG_S 6.0f
#define BALL_OUTER_BEAM_ACCEL_LIMIT_DEG_S2 30.0f
#define BALL_CONTROL_COMMAND_DELTA_DEG 0.05f
/* 外环是事件驱动的：每个新的视觉采样都会被评估。
 * 因此变化的角度指令可立即经 FC 下发；
 * 未变化的设定点有意不重复发送给 X42S。 */
#define BALL_CONTROL_COMMAND_MIN_PERIOD_MS 0U

/* 方向特定的单次启动冲击。它在连续 P 控制开始前
 * 撤销；它不是 I 项，也不会累积。 */
#define BALL_CONTROL_BREAKAWAY_TO_VISUAL_POS_DEG -7.0f
#define BALL_CONTROL_BREAKAWAY_TO_VISUAL_NEG_DEG 7.0f
#define BALL_CONTROL_BREAKAWAY_POS_REACHED_DEG -6.5f
#define BALL_CONTROL_BREAKAWAY_NEG_REACHED_DEG 6.5f
#define BALL_CONTROL_BREAKAWAY_SPEED_RPM 8.0f
#define BALL_CONTROL_BREAKAWAY_ACCEL_RPM_S 100U
#define BALL_CONTROL_BREAKAWAY_DECEL_RPM_S 100U
#define BALL_CONTROL_BREAKAWAY_MAX_CURRENT_MA 600U
#define BALL_CONTROL_BREAKAWAY_TIMEOUT_MS 800U
#define BALL_CONTROL_BREAKAWAY_HOLD_MS 120U
#define BALL_CONTROL_RETURN_TIMEOUT_MS 500U
#define BALL_CONTROL_RETURN_TOLERANCE_DEG 0.5f
#define BALL_CONTROL_ZERO_MOVE_TIMEOUT_MS 1500U
#define BALL_CONTROL_ZERO_RETRY_PERIOD_MS 100U

/* 正常 PD 阶段的静摩擦辅助。这有意不是
 * 积分项：当球在 1 cm 得分带之外但在测得的管体/凹槽
 * 摩擦点内停止时，保持已验证过的方向特定启动
 * 角度。在真实运动出现后释放它，随后由 PD 项
 * 提供预测性制动。 */
#define BALL_OUTER_STICTION_ERROR_CM 1.0f
#define BALL_OUTER_STICTION_ENTER_SPEED_CM_S 0.5f
#define BALL_OUTER_STICTION_RELEASE_SPEED_CM_S 0.8f
#define BALL_OUTER_STICTION_TO_VISUAL_POS_MOTOR_DEG -7.0f
#define BALL_OUTER_STICTION_TO_VISUAL_NEG_MOTOR_DEG 7.0f

/* 球开始运动后的低速连续角度跟踪。 */
/* 6 RPM = 36 deg/s：即使无斜坡，5 度也至少需要 139 ms。
 * 编码器稳定与机械稳定都需要测量。 */
#define BALL_CONTROL_MAX_SPEED_RPM 6.0f
#define BALL_CONTROL_ACCEL_RPM_S 100U
#define BALL_CONTROL_DECEL_RPM_S 100U
#define BALL_CONTROL_MAX_CURRENT_MA 600U

#define BALL_CONTROL_ENABLE_SETTLE_MS 300U
#define BALL_CONTROL_COMMAND_GAP_MS 100U
/* 参数帧 F1 必须先被接受，之后才能发送下一个 FC 目标帧，但
 * 启动后等待完整的 100 ms 会让球不必要地处于开环。 */
#define BALL_CONTROL_NORMAL_PARAM_GAP_MS 20U
/* 单个坏的检测/状态帧不得使横梁掉落。控制器保持其
 * 最后接受的测量/角度指令，直到超过该时间
 * 没有新的可用视觉采样。在 60 fps 下这可容忍约
 * 15 个连续坏帧，但仍能中止真实的摄像头/链路丢失。 */
#define BALL_CONTROL_VISION_TIMEOUT_MS 250U
/* 执行器位置每 50 ms 轮询一次。缺失的应答只有在
 * 驱动的 100 ms 应答超时之后才重试，因此有效的重试可能在上一次
 * 反馈后约 160 ms 到达。保持该值高于一个重试间隔；
 * 否则正常的 UART 重试会被误判为执行器丢失。 */
#define BALL_CONTROL_ANGLE_TIMEOUT_MS 800U
/* 摄像头坐标是量化的。不要用单帧微分来
 * 判定球静止：60 fps 下一个像素看起来就像很大的
 * 虚假速度。而应要求所有位置采样在指定持续时间内
 * 都保持在这个小窗口内。 */
#define BALL_CONTROL_STILL_POSITION_BAND_CM 0.25f
#define BALL_CONTROL_STILL_WINDOW_MS 500U
#define BALL_CONTROL_SETTLE_HOLD_MS 300U
#define BALL_CONTROL_RUN_TIMEOUT_MS 20000U

/* 在 P 已证实起作用但球仍处于得分带外时，用于
 * 脱离局部凹槽摩擦点的有界脉冲。 */
#define BALL_CONTROL_REKICK_ERROR_CM 1.0f
#define BALL_CONTROL_REKICK_MAX_COUNT 3U

#endif

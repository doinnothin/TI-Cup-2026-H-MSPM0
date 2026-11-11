#ifndef Q3_CONTROL_CONFIG_H
#define Q3_CONTROL_CONFIG_H

/* 在本车上测得的机械事实。将它们集中保存，使后续
 * 整定永远不会把硬件知识隐藏在状态机代码里。 */
#define Q3_BEAM_LENGTH_MM 250.0f
#define Q3_MOTOR_SAFE_LIMIT_DEG 25.0f

/* 已确认的符号约定：
 *   +电机角度 -> 右端上升 -> 球朝视觉负方向发展。
 * 因此外环位置控制器需要负的被控对象符号。 */
#define Q3_POSITION_TO_MOTOR_SIGN -1.0f

/* 在 8 rpm 下测得的方向特定动态启动值。
 * 这些是短时前馈助推，绝不是连续的 PID 输出。 */
#define Q3_BREAKAWAY_TO_VISUAL_NEG_DEG 7.0f
#define Q3_BREAKAWAY_TO_VISUAL_POS_DEG -7.0f
#define Q3_BREAKAWAY_SPEED_RPM 8.0f

/* 首轮外环约束。生效的位置 PID 增益与
 * 启动值位于 ball_control_config.h；本文件把测得的
 * 机械限幅与定时常量集中在一处。 */
#define Q3_OUTER_LOOP_PERIOD_MS 10U
#define Q3_ACTUAL_ANGLE_FEEDBACK_PERIOD_MS 50U
#define Q3_VISION_TIMEOUT_MS 250U
#define Q3_NORMAL_ANGLE_LIMIT_DEG 8.0f

/* 正式最终条件是被测的球位置，而不是最后一个
 * 0.1 cm 内部参考步进：球必须静止在 -5 +/- 1 cm 以内。
 * BallPositionControl 的 target_reached 标志提供速度与
 * 静止性证明；此处定义独立的得分位置带。 */
#define Q3_FINAL_ACCEPTANCE_TOLERANCE_CM 1.0f

/* 一旦记录该最终条件，就冻结秒表并立即
 * 将 X42S 会话目标指令回到 PA31 确立的零点。稳定窗口
 * 测试已在本状态之前完成，因此再进行一次定时
 * 保持只会让 UI 看起来像没有停下。 */
#define Q3_FINAL_HOLD_MS 0U
#define Q3_RETURN_ZERO_TIMEOUT_MS 1500U
/* 当首次 PA31 准备请求在复位或有意禁用后等待
 * 新的 X42S 选项反馈时，限制重新使能/置零写入。 */
#define Q3_ZERO_PREPARE_RETRY_MS 200U

/* 快速 +5 -> -5 过渡。比赛序列不要求在 +5 处保持；
 * +/-1.0 cm 带内的一个有效视觉采样即可触发反向。
 * 将其与最终位置容差分开：它防止通用外环控制器的
 * HOLD 状态在静摩擦仍存在时使 Q3 序列
 * 卡在 +4.2 cm 附近。 */
#define Q3_PLUS5_SWITCH_TOLERANCE_CM 1.0f
#define Q3_PLUS5_CONFIRM_FRAMES 1U

/* 返回曲线：内部参考连续移动，而不是
 * 指令一个 10 cm 的阶跃。中段巡航更快；参考与
 * 输出在预测制动区内都会减速。 */
#define Q3_RETURN_FINAL_TARGET_CM -5.0f
#define Q3_RETURN_REFERENCE_UPDATE_MS 20U
#define Q3_RETURN_CRUISE_REFERENCE_SPEED_CM_S 14.0f
#define Q3_RETURN_BRAKE_REFERENCE_SPEED_CM_S 3.0f
#define Q3_RETURN_BRAKE_REFERENCE_START_CM -2.5f
#define Q3_RETURN_FEEDBACK_DELAY_MS 150U
#define Q3_RETURN_DECELERATION_CM_S2 70.0f
#define Q3_RETURN_BRAKE_MARGIN_CM 0.5f
#define Q3_RETURN_BRAKE_FALLBACK_DISTANCE_CM 1.0f

#endif

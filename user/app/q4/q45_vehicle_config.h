#ifndef Q45_VEHICLE_CONFIG_H
#define Q45_VEHICLE_CONFIG_H

#include "q4_control_config.h"

/*
 * 第 4、5 问的车辆路径配置。
 *
 * Q4 静态恢复增益保留在 q4_control_config.h 中。下面所有动态车辆补偿
 * 均归 Q5 所有，使 Q5 调参不会悄然改变已验证的 Q3 或静态 Q4 控制环。
 */
#define Q45_POSITION_TO_MOTOR_SIGN -1.0f

/* Q5 动态模型。导轨横跨车辆安装，因此测得的侧向加速度 v^2/R 是主要的
 * 转弯扰动。本车没有陀螺仪，所以曲率来自固定的 1.5 m 直道 / 0.5 m 半径
 * 椭圆几何与路径里程计。两个半圆的曲率相反：在导轨坐标系中第一个为
 * 负、第二个为正。COM12 证实：正的第一次转弯补偿会使继承的静态 PD
 * 立即与其对抗，因此符号应放入路径模型中，而不是对两个弯道统一取反。 */
#define Q45_ACCELERATION_FEEDFORWARD_ENABLED 1U
#define Q45_Q5_GRAVITY_M_S2 9.80665f
/* 静态平衡：g*sin(theta) = a_parallel*cos(theta)。球体的滚动惯量会同时
 * 乘到两项上，并在加速度为零时相互抵消。 */
/* 三次完整的 Q5 轨迹在加速的第一段直道中漂移了 +1.9--3.5 cm，而该项
 * 只提供了 0.10--0.20 度。此处采用完整的实测 a/g 投影；转弯补偿仍然独立。 */
#define Q45_Q5_LONGITUDINAL_FF_GAIN 1.00f
/* 起步时，COM12 显示加速度模型要求 +0.2--0.5 度，而小球已经向后移动。
 * 先让不变的静态 Q4 环建立出发平衡，然后在距 150 cm 第一个弯道之前
 * 充分恢复已验证的完整纵向模型。 */
#define Q45_Q5_LONGITUDINAL_FF_RAMP_DISTANCE_CM 60.0f
/* 当前 60 cm 内 f100=0 时，小球仍保持在 +1.8--2.0 cm。这个仅 Q5 使用的
 * 乘子只在第一个弯道之前增强继承的静态修正；它不改变 Q3、静态 Q4 或弯道调参。 */
#define Q45_Q5_LAUNCH_STATIC_BEAM_GAIN 1.40f
/* COM12：在 300 mm/s 时，几何 1.05 度的满量程项会在继承的静态 PD 稳定
 * 小球之前就把它反向推过中心。保留其已验证的符号，但只施加 55%
 * （巡航时约 0.58 度），把已验证的 Q4 PD 权限留给残差。 */
#define Q45_Q5_LATERAL_FF_GAIN 0.55f
/* 第一个弯道：Q5D 显示小球越过中心后，存在 -0.55--0.65 度的稳态前馈
 * 与 Q4 刹车相对抗。匹配独立验证过的第二个弯道水平，同时保持每个弯道
 * 可单独调参。 */
#define Q45_Q5_FIRST_TURN_LATERAL_FF_GAIN 0.30f
/* 24 cm 的第二转弯斜坡消除了进入台阶，但其 0.6 度的平台仍然抵消继承的
 * Q4 刹车并把小球推向负方向。只减小第二个弯道；第一次转弯补偿保持不变。 */
#define Q45_Q5_SECOND_TURN_LATERAL_FF_GAIN 0.15f
#define Q45_Q5_FEEDFORWARD_GAIN 1.00f
#define Q45_Q5_FEEDFORWARD_BEAM_SIGN 1.00f
#define Q45_Q5_FEEDFORWARD_LIMIT_DEG 1.60f
#define Q45_Q5_PARALLEL_ACCEL_LIMIT_M_S2 0.40f
#define Q45_Q5_SPEED_FILTER_ALPHA 0.30f
#define Q45_Q5_ACCEL_FILTER_ALPHA 0.20f
#define Q45_Q5_CHASSIS_SAMPLE_MS 20U
/* Q5 日志显示出现孤立的 260--280 ms 相机帧间隙，随后是有效跟踪。
 * 保持静态 Q4 就绪看门狗不变；仅 Q5 获得足够余量以度过一次相机调度
 * 间隙，而持续丢失仍会安全停车。 */
#define Q45_Q5_VISION_TIMEOUT_MS 600U
#define Q45_Q5_TURN_RADIUS_M 0.50f
#define Q45_Q5_FIRST_TURN_START_CM 150.0f
/* 0.15 s 的路径预览在物理出口之前就释放了第一弯道项。将其保留到第一段
 * 直道内 8 cm 处；保持第二个弯道独立。 */
#define Q45_Q5_FIRST_TURN_END_CM 315.08f
#define Q45_Q5_SECOND_TURN_START_CM 457.08f
#define Q45_Q5_SECOND_TURN_END_CM 614.16f
#define Q45_Q5_FIRST_TURN_ACCEL_SIGN -1.0f
#define Q45_Q5_SECOND_TURN_ACCEL_SIGN 1.0f
#define Q45_Q5_TURN_BLEND_DISTANCE_CM 6.0f
/* Q5D 显示第二转弯前馈在 0.2 s 内从 0 跳到 0.46 度，快于 X42S 的跟随
 * 能力。只拓宽该转弯；第一次转弯的调参保持不变。 */
#define Q45_Q5_SECOND_TURN_BLEND_DISTANCE_CM 24.0f
/* 最终 Q5 路径标定：为每个弯道保留各自独立的进入、顶点与退出配置，
 * 之后接有界的视觉反馈修正。 */
#define Q45_Q5_FIRST_TURN_ENTRY_LATERAL_FF_GAIN 0.40f
#define Q45_Q5_SECOND_TURN_ENTRY_LATERAL_FF_GAIN 0.20f
#define Q45_Q5_FIRST_TURN_ENTRY_GAIN_HOLD_CM 125.0f
#define Q45_Q5_SECOND_TURN_ENTRY_GAIN_HOLD_CM 125.0f
#define Q45_Q5_TURN_ENTRY_GAIN_FADE_CM 40.0f
#define Q45_Q5_FIRST_TURN_APEX_RAMP_START_CM 10.0f
#define Q45_Q5_FIRST_TURN_APEX_RAMP_FULL_CM 35.0f
#define Q45_Q5_FIRST_TURN_APEX_FADE_START_CM 85.0f
#define Q45_Q5_FIRST_TURN_APEX_FADE_END_CM 125.0f
#define Q45_Q5_SECOND_TURN_APEX_RAMP_START_CM 5.0f
#define Q45_Q5_SECOND_TURN_APEX_RAMP_FULL_CM 35.0f
#define Q45_Q5_SECOND_TURN_APEX_FADE_START_CM 125.0f
#define Q45_Q5_SECOND_TURN_APEX_FADE_END_CM 155.0f
#define Q45_Q5_FIRST_TURN_APEX_EXTRA_GAIN 0.14f
#define Q45_Q5_SECOND_TURN_APEX_EXTRA_GAIN 0.00f
#define Q45_Q5_SECOND_TURN_FF_POLARITY -1.0f
#define Q45_Q5_TURN_PREVIEW_S 0.15f

#define Q45_Q5_FIRST_FB_START_CM 155.0f
#define Q45_Q5_FIRST_FB_FULL_CM 175.0f
#define Q45_Q5_FIRST_FB_FADE_START_CM 290.0f
#define Q45_Q5_FIRST_FB_END_CM 315.0f
#define Q45_Q5_FIRST_FB_DEADBAND_CM 0.60f
#define Q45_Q5_FIRST_FB_KP_DEG_PER_CM 0.090f
#define Q45_Q5_FIRST_FB_KD_DEG_PER_CM_S 0.060f
#define Q45_Q5_FIRST_FB_D_LIMIT_DEG 0.18f
#define Q45_Q5_FIRST_FB_LIMIT_DEG 0.25f
#define Q45_Q5_FIRST_FB_CONF_MIN 80U
#define Q45_Q5_FIRST_TRIAL_ABORT_CM 4.50f
#define Q45_Q5_REVERSED_SECOND_TEST_ABORT_CM 4.50f
#define Q45_Q5_SECOND_FB_START_CM 440.0f
#define Q45_Q5_SECOND_FB_FULL_CM 460.0f
#define Q45_Q5_SECOND_FB_END_CM 622.16f
#define Q45_Q5_SECOND_FB_DEADBAND_CM 0.60f
#define Q45_Q5_SECOND_FB_KP_DEG_PER_CM 0.13f
#define Q45_Q5_SECOND_FB_P_LIMIT_DEG 0.55f
#define Q45_Q5_SECOND_FB_KI_DEG_PER_CM_S 0.07f
#define Q45_Q5_SECOND_FB_I_LIMIT_DEG 0.30f
#define Q45_Q5_SECOND_FB_I_MAX_SPEED_CM_S 2.00f
#define Q45_Q5_SECOND_FB_I_CONF_MIN 80U
#define Q45_Q5_SECOND_FB_I_LEAK_PER_S 1.50f
#define Q45_Q5_TURN_STICTION_DISABLE_LAT_M_S2 0.08f
#define Q45_Q5_LATE_STRAIGHT_STATIC_GAIN 1.08f
#define Q45_Q5_FIRST_STRAIGHT_LATE_START_CM 75.0f
#define Q45_Q5_SECOND_STRAIGHT_LATE_START_CM 386.08f
#define Q45_Q5_LATE_STRAIGHT_GAIN_BLEND_CM 25.0f

/* Q5 专属小球环，以物理梁角度表示。P/D 修正模型前馈之后的残差。
 * I 有意设置得缓慢且有界；它只在中心附近学习很小的残余安装偏置。 */
/* 从当前最终的 Q5 基线复制的 Q5 专属动态小球控制数值；Q4 与 Q6 继续
 * 使用其现有的按模式区分的路径。 */
#define Q45_Q5_KP_BEAM_DEG_PER_CM 0.20f
#define Q45_Q5_KP_ERROR_POS 0.23f
#define Q45_Q5_KD_BEAM_DEG_PER_CM_S 0.20f
#define Q45_Q5_KD_ERROR_POS 0.20f
#define Q45_Q5_BRAKE_KP 0.12f
#define Q45_Q5_BRAKE_KD 0.12f
#define Q45_Q5_BRAKE_KD_OUTWARD 0.16f
#define Q45_Q5_CRUISE_KD_OUTWARD 0.20f
#define Q45_Q5_HOLD_KP 0.10f
#define Q45_Q5_HOLD_KD 0.12f
#define Q45_Q5_CRUISE_BEAM_LIMIT_DEG 1.80f
#define Q45_Q5_CRUISE_BEAM_LIMIT_POS_DEG 2.20f
#define Q45_Q5_CRUISE_BEAM_LIMIT_NEG_DEG 2.00f
#define Q45_Q5_BRAKE_BEAM_LIMIT_DEG 1.80f
#define Q45_Q5_BRAKE_BEAM_LIMIT_NEG_DEG 1.80f
#define Q45_Q5_HOLD_BEAM_LIMIT_DEG 0.50f
#define Q45_Q5_MID_DISTANCE_CM 5.00f
#define Q45_Q5_MID_BEAM_LIMIT_DEG 1.60f
#define Q45_Q5_HOLD_POSITION_TOLERANCE_CM 1.00f
#define Q45_Q5_HOLD_SPEED_LIMIT_CM_S 3.00f
#define Q45_Q5_BRAKE_FALLBACK_CM 7.00f
#define Q45_Q5_BRAKE_FALLBACK_POS_CM 5.00f
#define Q45_Q5_BRAKE_MARGIN_PROFILE_CM 2.00f
#define Q45_Q5_FEEDBACK_DELAY_MS 150U
#define Q45_Q5_DECELERATION_CM_S2 70.0f
#define Q45_Q5_LEAD_TIME_S 0.25f
#define Q45_Q5_LEAD_TIME_ERROR_NEG_S 0.30f
/* Q5 专属慢速修正：三次完整轨迹在底盘加速度降为零之后仍保留 +1.6--2.4 cm。
 * 这不属于静态 Q4。 */
/* Q5D 证明修正符号正确，但在第一个弯道之前只累积了 +0.06--0.12 度。
 * 将收敛速度加倍，同时保留现有的 0.25 度上限与所有门控条件。 */
#define Q45_Q5_KI_BEAM_DEG_PER_CM_S 0.070f
#define Q45_Q5_INTEGRAL_LIMIT_DEG 0.25f
#define Q45_Q5_INTEGRAL_LEAK_PER_S 0.10f
#define Q45_Q5_INTEGRAL_ACTIVE_POSITION_CM 1.00f
#define Q45_Q5_INTEGRAL_ACTIVE_SPEED_CM_S 2.00f
#define Q45_Q5_INTEGRAL_ACTIVE_ACCEL_M_S2 0.02f
#define Q45_Q5_D_TERM_LIMIT_DEG 0.80f
#define Q45_Q5_TOTAL_BEAM_LIMIT_DEG 2.20f
#define Q45_Q5_BEAM_RATE_LIMIT_DEG_S 6.00f
#define Q45_Q5_BEAM_ACCEL_LIMIT_DEG_S2 30.0f
#define Q45_Q5_COMMAND_LEAD_LIMIT_DEG 0.70f
#define Q45_Q5_COMMAND_PERIOD_MS Q4_RECOVERY_COMMAND_PERIOD_MS
/* X42S 位置回复是异步诊断反馈，而非命令应答。COM12 轨迹显示正常回复
 * 可能相隔 300 ms，而 FOC 链路仍在线并跟随目标。取值为 0 时，将丢链
 * 保护交由执行器既有的在线看门狗处理，与已验证的 Q4 静态恢复路径一致。 */
#define Q45_Q5_ACTUAL_ANGLE_TIMEOUT_MS 0U
#define Q45_Q5_PREDICTION_TIME_S 0.18f

/* 评分与车辆运动相互独立。越过 +/-1 cm 会锁存一次失败运行，但不会切换
 * 底盘速度。只有路径/时间/故障/操作员终止才会使底盘停车，且终止状态
 * 从不自动重启。 */
#define Q45_Q5_INTERNAL_POSITION_CM 0.35f
#define Q45_Q5_INTERNAL_SPEED_CM_S 1.00f
#define Q45_Q5_HOLD_BLEND_END_CM 0.65f
#define Q45_Q5_HOLD_BLEND_END_SPEED_CM_S 2.00f
#define Q45_Q5_BRAKE_MARGIN_CM 0.15f
#define Q45_Q5_BRAKE_BLEND_CM 0.30f

/* 复用实测的执行器基线，同时固定控制调度。 */
#define Q45_Q5_ACTUATOR_ACCEL_RPM_S Q4_ACCEL_RPM_S
#define Q45_Q5_ACTUATOR_DECEL_RPM_S Q4_DECEL_RPM_S
#define Q45_Q5_ACTUATOR_MAX_SPEED_RPM Q4_MAX_SPEED_RPM
#define Q45_Q5_ACTUATOR_MAX_CURRENT_MA Q4_MAX_CURRENT_MA
#define Q45_Q5_CONFIDENCE_MIN 60U

/* A->B 为 1.50 m。5.0 s 斜坡在约 7.5 s 的正式车轮运动中到达 B，从而在
 * 第二次按键开始的 8 s 秒表内留有余量。电机准备与人工放置小球在此前完成。 */
#define Q45_Q4_CRUISE_SPEED_MM_S 300.0f
#define Q45_Q4_ACCEL_TIME_MS 5000U
#define Q45_Q4_B_DISTANCE_CM 150.0f
#define Q45_Q4_STOP_DISTANCE_CM 180.0f
#define Q45_Q4_TIME_LIMIT_MS 8000U

/* 一圈为 2*1.5 m + 2*pi*0.5 m = 6.1416 m。越过 A 后继续 8 cm，使车辆在
 * 常规底盘刹车之前明确通过标记。 */
#define Q45_Q5_CRUISE_SPEED_MM_S 300.0f
#define Q45_Q5_ACCEL_TIME_MS 6000U
#define Q45_Q5_A_DISTANCE_CM 614.16f
#define Q45_Q5_STOP_DISTANCE_CM 622.16f
#define Q45_Q5_TIME_LIMIT_MS 30000U

/* 电机会话在车辆运动之前准备。这些延时保留了静态 Q4 控制器中已验证的
 * X42S F1->FC 顺序。 */
#define Q45_ACTUATOR_READY_TIMEOUT_MS 1500U
#define Q45_ENABLE_TO_PROFILE_GAP_MS 100U
#define Q45_PROFILE_TO_ZERO_GAP_MS 20U
#define Q45_ZERO_TIMEOUT_MS 1000U
/* Q6 在每次试验之前都会物理返回到电机存储的绝对零点，因此其调零阶段
 * 超时必须覆盖回程：一次停在约 14 度的运行约需一秒回程外加斜坡余量。 */
#define Q45_Q6_ZERO_RETURN_TIMEOUT_MS 5000U

/* 安全/看门狗限值采用与静态 Q4 相同的中心与新鲜帧语义；竞赛要求本身
 * 仍为 |x| <= 1 cm。 */
#define Q45_VISION_TIMEOUT_MS Q4_VISUAL_READY_AGE_MS
#define Q45_CENTER_POSITION_TOLERANCE_CM Q4_HOLD_POSITION_TOLERANCE_CM
#define Q45_CENTER_SPEED_LIMIT_CM_S Q4_HOLD_SPEED_LIMIT_CM_S
#define Q45_BALL_SAFETY_POSITION_CM 11.0f

/* 只有在实测的静止误差持续存在后，才提供有限的 4 度 Q4 脉冲。它既不是
 * 加速度前馈也不是积分器。 */
#define Q45_STICTION_CONFIRM_MS 500U
#define Q45_STICTION_HOLD_MS Q4_BREAKAWAY_HOLD_MS
#define Q45_STICTION_MAX_COUNT 3U

#endif

#ifndef Q6_CONFIG_H
#define Q6_CONFIG_H

/* Q6 目标/捕获与控制器自适应参数。Q3/Q4/Q5 整定
 * 表仍由其原有模块维护，此处未作改动。 */
#define Q6_CAPTURE_MIN_FRAMES 15U
#define Q6_CAPTURE_MIN_SPAN_MS 500U
#define Q6_CAPTURE_MAX_FRAME_GAP_MS 100U
#define Q6_CAPTURE_MAX_AGE_MS 100U
#define Q6_CAPTURE_MIN_CONFIDENCE 60U
#define Q6_CAPTURE_MAX_POSITION_RANGE_CM 0.20f
#define Q6_CAPTURE_MAX_SPEED_CM_S 0.50f
#define Q6_EXPLICIT_START_TOLERANCE_CM 1.00f
#define Q6_TEST_TARGET_LIMIT_CM 5.00f
/* 保留的旧阈值；当前 Q6 积分与静摩擦几何改用
 * Q6_STICTION_ENGAGE_CM 代替。 */
#define Q6_HOLD_ENGAGE_TOLERANCE_CM 0.20f
/* 全长 250 mm 的物理管道，以标定原点为中心。 */
#define Q6_DYNAMIC_CAPTURE_MAX_POSITION_CM 12.50f
#define Q6_EXPLICIT_CAPTURE_MAX_POSITION_CM Q6_DYNAMIC_CAPTURE_MAX_POSITION_CM
/* Q6 横梁偏置的最大学习贡献量，单位为度。 */
#define Q6_INTEGRAL_LIMIT_DEG 0.80f
/* Q6 专用的有界静摩擦推压重试次数预算。 */
#define Q6_STICTION_MAX_COUNT 10U
/* 有界 Q6 静摩擦脉冲对应的电机侧角度幅值。 */
#define Q6_STICTION_PUSH_MOTOR_DEG 10.0f
/* Q6 静摩擦辅助的位置误差阈值与脉冲持续时间。
 * 参数仅用于 Q6；Q4/Q5 的时序与门限保持不变。 */
#define Q6_STICTION_ENGAGE_CM 0.55f
#define Q6_STICTION_PUSH_HOLD_MS 200U
/* Q6 BRAKE 阻尼增益，单位 beam-deg 每 (cm/s)，包含外行运动
 * 分支。Q4/Q5 继续使用其冻结值。 */
#define Q6_BRAKE_KD_BEAM_DEG_PER_CM_S 0.18f
#define Q6_BRAKE_KD_OUTWARD_BEAM_DEG_PER_CM_S 0.24f
/* 在长目标距离时将 BRAKE 选为 Q6 配置回退；这
 * 可避免为 Q6 选中共享的 CRUISE 进入律。 */
#define Q6_BRAKE_FALLBACK_CM 18.00f
/* Q6 专用的视觉准入带。带外样本会被
 * Q6 消费端忽略，持续被拒仍受新鲜度
 * 看门狗约束；视觉协议/解析器保持不变。 */
#define Q6_BALL_PIXEL_CY_CENTER 165U
#define Q6_BALL_PIXEL_CY_BAND 60U
/* 为静态试验工具预留的验收阈值。当前
 * 运行时控制路径不消费这些值。 */
#define Q6_STATIC_SETTLE_MS 2000U
#define Q6_STATIC_HOLD_MS 10000U
#define Q6_STATIC_MAX_ERROR_CM 0.50f
#define Q6_STATIC_MIN_FRAMES 100U
#define Q6_STATIC_REPEAT 3U
#define Q6_STATIC_ABORT_DEVIATION_CM 3.00f
/* Q6 任务的总开关；使能时 PB24 映射到 Q6。 */
#ifndef Q6_TASK_ENABLE
#define Q6_TASK_ENABLE 1U
#endif
/* 台架图 1：底盘驻车时的保持位置试验。 */
#ifndef Q6_STATIC_TEST_MODE
#define Q6_STATIC_TEST_MODE 0U
#endif
/* 台架图 2（默认关闭）：一旦选中 Q6，捕获到的目标可
 * 无需正常的第二次 Q6 启动键按下即发射。 */
#ifndef Q6_BENCH_AUTO_START
#define Q6_BENCH_AUTO_START 0U
#endif
/* 台架图 3（默认关闭）：启动后一秒自动进入
 * Q6，使烧录复位后无需任何按键即可开始整个试验。 */
#ifndef Q6_BOOT_AUTO_SELECT
#define Q6_BOOT_AUTO_SELECT 0U
#endif
/* 台架图 4（默认关闭）：将启动自动试验的保持目标固定为
 * 以毫米表示的显式值（50 = +5.0 cm）。哨兵值 -32768 使
 * 试验保持在 Q6_TARGET_CAPTURE 模式（锁定小球静止处）。 */
#ifndef Q6_BENCH_EXPLICIT_TARGET_MM
#define Q6_BENCH_EXPLICIT_TARGET_MM (-32768)
#endif

#endif

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* 用于 Q2 路线跟随与车辆任务的底盘运动控制外观层。
 * 速度参数使用 mm/s；航向参数使用角度。定时器回调
 * 执行上层控制环，因此只有 Control_QuickBrake() 适合
 * 在该中断上下文中即时调用。Control_Stop() 采用忙等待，
 * 且仅限前台使用。
 */

/* 速度内环参数。 */
#define SPEED_P 5.0f
#define SPEED_I 0.5f
#define SPEED_D 0.30f

/* 上层环参数。 */
#define YAW_P 15.0f
#define YAW_D 4.0f
#define YAW_K 0.50f
#define YAW_DEADBAND 0.8f
#define TRACE_P 12.0f
#define TRACE_D 4.0f
#define TRACE_FILTER_ALPHA 0.35f

#define MOTOR_TARGET_MAX 800.0f
#define STOP_BRAKE_MS 80U

#define SENSOR_ALL_WHITE 0x00U
#define WHITE_CONFIRM_COUNT 10U
#define BLACK_CONFIRM_COUNT 10U
#define STRAIGHT_GUARD_COUNT 50U

#define CONTROL_EVENT_NONE 0U
#define CONTROL_EVENT_BLACK 1U
#define CONTROL_EVENT_WHITE 2U

extern volatile uint8_t control_enable;
extern volatile uint8_t control_auto_enable;
extern volatile uint8_t control_yaw_enable;
extern volatile uint8_t control_trace_enable;
extern volatile float control_target_yaw;
extern volatile float control_trace_error;

/* 以请求的航向（deg）和基础速度（mm/s）启动轮速控制；
 * 在选择前上层环保持禁用。 */
void Control_Start(float yaw, float speed);
/* 有界的 PWM/GPIO 短制动动作；对现有定时器 ISR 安全。 */
void Control_QuickBrake(void);
/* 先停止再释放电机输出；因使用 delay_ms 故仅限前台。 */
void Control_Stop(void);
void Control_YawStart(float yaw, float speed);
void Control_AutoStart(float speed);
void Control_TraceStart(float speed);
void Control_SetBaseSpeed(float speed);

/* 执行一次上层环更新。这些函数由现有控制定时器 ISR
 * 调用；当检测到相应的线状态跳变时返回 1。 */
uint8_t Control_StraightRun(void);
uint8_t Control_TraceRun(void);
void Control_ToTrace(void);
void Control_ToStraight(void);
void Control_UpperRun(void);
uint8_t Control_GetEvent(void);

/* 使能并启动现有 SysConfig 控制定时器。 */
void Control_TimerInit(void);

#endif

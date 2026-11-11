#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 固定状态的 PID 基本运算与现有底盘速度环状态。
 * pid_control() 由编码器速度定时器 ISR 运行；各计算辅助函数
 * 就地更新调用方拥有的 pid_ctrl_t，且不分配内存。
 */

enum
{
    POSITION_PID = 0,
    DELTA_PID
};

typedef struct
{
    float target;
    float now;
    float error[3];
    float p;
    float i;
    float d;
    float pout;
    float iout;
    float dout;
    float out;
    uint32_t pid_mode;
} pid_ctrl_t;

/* 三类控制环：速度环、航向环、灰度环。 */
extern pid_ctrl_t motorA;
extern pid_ctrl_t motorB;
extern pid_ctrl_t angle;
extern pid_ctrl_t trace_pid;

/* 初始化控制器状态与增益。 */
void pid_init(pid_ctrl_t *pid, uint32_t mode, float p, float i, float d);
void pid_cal(pid_ctrl_t *pid);
void pid_cal_angle(pid_ctrl_t *pid);
void pid_cal_trace(pid_ctrl_t *pid);
void pidout_limit(pid_ctrl_t *pid);

/* 由现有速度定时器 ISR 调用；读取编码器速度并写入 PWM。 */
void pid_control(void);
void motor_target_set(float left, float right);

/* 计算最短航向角误差。 */
float Yaw_error_zzk(float target, float now);

#endif

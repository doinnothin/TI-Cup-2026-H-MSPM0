#include "pid.h"
#include "encoder.h"
#include "motor.h"

#define PWM_MAX 1600.0f
#define IOUT_MAX 800.0f
#define IOUT_MIN -800.0f

pid_ctrl_t motorA;
pid_ctrl_t motorB;
pid_ctrl_t angle;
pid_ctrl_t trace_pid;

void pid_init(pid_ctrl_t *pid, uint32_t mode, float p, float i, float d)
{
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->target = 0.0f;
    pid->now = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
}

static void Pid_Calculate(pid_ctrl_t *pid, float error)
{
    pid->error[0] = error;

    if (pid->pid_mode == DELTA_PID)
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;

        if (pid->out > PWM_MAX)
        {
            pid->out = PWM_MAX;
            pid->iout = IOUT_MAX;
        }
        if (pid->out < -PWM_MAX)
        {
            pid->out = -PWM_MAX;
            pid->iout = IOUT_MIN;
        }
    }
    else
    {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

void pid_cal(pid_ctrl_t *pid)
{
    Pid_Calculate(pid, pid->target - pid->now);
}

void pid_cal_angle(pid_ctrl_t *pid)
{
    Pid_Calculate(pid, Yaw_error_zzk(pid->target, pid->now));
}

void pid_cal_trace(pid_ctrl_t *pid)
{
    Pid_Calculate(pid, pid->target - pid->now);
}

void pidout_limit(pid_ctrl_t *pid)
{
    if (pid->out > PWM_MAX)
    {
        pid->out = PWM_MAX;
    }
    if (pid->out < -PWM_MAX)
    {
        pid->out = -PWM_MAX;
    }
}

void pid_control(void)
{
    motorA.now = E_speed[MOTOR_LEFT];
    motorB.now = E_speed[MOTOR_RIGHT];

    pid_cal(&motorA);
    pid_cal(&motorB);
    pidout_limit(&motorA);
    pidout_limit(&motorB);

    Motor_Setduty(MOTOR_LEFT, (int16_t)motorA.out);
    Motor_Setduty(MOTOR_RIGHT, (int16_t)motorB.out);
}

void motor_target_set(float left, float right)
{
    motorA.target = left;
    motorB.target = right;
}

float Yaw_error_zzk(float target, float now)
{
    float error = target - now;

    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

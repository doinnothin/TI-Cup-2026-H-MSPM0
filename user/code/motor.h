#ifndef MOTOR_H
#define MOTOR_H

#include "headfile.h"

/* 双通道底盘 H 桥/PWM 适配层。占空比为带符号值，由驱动钳位到所配置的
 * PWM 周期；零为惰行，Motor_Brake() 施加既有的短路刹车引脚状态。
 * 调用执行有界的寄存器写操作。
 */

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0
#define MOTOR_STOPS 2

void Motor_Init(void);
/* 带符号 PWM 指令：正为前进，负为后退，零为惰行。 */
void Motor_Setduty(uint8_t motor, int16_t speed);
void Motor_Stop(void);
void Motor_Brake(void);

#endif

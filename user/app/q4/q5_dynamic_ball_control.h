#ifndef Q5_DYNAMIC_BALL_CONTROL_H
#define Q5_DYNAMIC_BALL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* 有状态的 Q5 球控制器，与外设无关。输入使用 cm、
 * cm/s、m/s^2、秒与电机角度；输出使用光束/电机角度
 * 与预测的 cm。Update 会修改调用方持有的控制器状态，并在
 * 输入无效或几何结果无法表示时返回 false。
 */
typedef struct
{
    float beam_angle_deg;
    float beam_rate_deg_s;
    float integral_beam_deg;
    bool initialized;
} Q5DynamicBallController;

typedef struct
{
    float measured_position_cm;
    float error_cm;
    float velocity_cm_s;
    float parallel_accel_m_s2;
    float dt_s;
    float actual_motor_deg;
} Q5DynamicBallInput;

typedef struct
{
    float p_beam_deg;
    float d_beam_deg;
    float i_beam_deg;
    float feedforward_beam_deg;
    float raw_beam_deg;
    float limited_beam_deg;
    float shaped_beam_deg;
    float predicted_position_cm;
    float motor_deg;
    uint8_t profile; /* 0 近中心，1 驱动，2 制动（平滑混合）。 */
    bool limited;
} Q5DynamicBallOutput;

void Q5DynamicBallControl_Init(Q5DynamicBallController *controller);
/* 计算一个有界控制步；不访问 UART、PWM 或全局状态。 */
bool Q5DynamicBallControl_Update(Q5DynamicBallController *controller,
                                 const Q5DynamicBallInput *input,
                                 Q5DynamicBallOutput *output);

#endif

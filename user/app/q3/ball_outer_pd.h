#ifndef BALL_OUTER_PD_H
#define BALL_OUTER_PD_H

#include <stdbool.h>

typedef enum
{
    BALL_OUTER_PROFILE_NORMAL = 0,
    BALL_OUTER_PROFILE_RETURN_CRUISE,
    BALL_OUTER_PROFILE_RETURN_BRAKE,
    BALL_OUTER_PROFILE_RETURN_HOLD
} BallOuterPdProfile;

typedef struct
{
    float beam_angle_deg;
    float beam_rate_deg_s;
    bool stiction_assist_active;
    bool initialized;
    BallOuterPdProfile profile;
} BallOuterPd;

typedef struct
{
    float p_deg;
    float d_deg;
    float raw_beam_deg;
    float limited_beam_deg;
    float shaped_beam_deg;
    float beam_rate_deg_s;
    float end_height_mm;
    float motor_deg;
    float stiction_motor_deg;
    float predicted_stop_cm;
    bool braking;
    bool stiction_assist;
    bool limited;
} BallOuterPdOutput;

void BallOuterPd_Init(BallOuterPd *controller);
void BallOuterPd_ResetTrajectory(BallOuterPd *controller);
void BallOuterPd_SetProfile(BallOuterPd *controller, BallOuterPdProfile profile);

/* 位置 P 与估计速度 D 直接请求物理横梁角度。
 * X42S 已闭合自身的电机角度环；本 MCU 不得基于同一摄像头
 * 测量再臆造一个速度 PID。 */
bool BallOuterPd_Update(BallOuterPd *controller,
                        float error_cm,
                        float velocity_cm_s,
                        float dt_s,
                        float actual_motor_deg,
                        BallOuterPdOutput *output);

#endif

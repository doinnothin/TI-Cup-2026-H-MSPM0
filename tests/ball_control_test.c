#include <math.h>
#include <string.h>
#include "app/q3/ball_position_control.h"
#include "app/q3/q3_sequence.h"
#include "app/ball_actuator/ball_actuator.h"
#include "code/vision_link.h"
#include "app/q3/ball_outer_pd.h"
#include "app/q3/ball_state_estimator.h"
#include "app/q3/pendulum_geometry.h"
static VisionLinkSample vision;
static BallActuatorStatus motor;
static float last_command;
static bool motor_enabled;
static unsigned failures;
static unsigned checks;
static unsigned first_failure_line;
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        ++checks;                                                                                  \
        if (!(x))                                                                                  \
        {                                                                                          \
            ++failures;                                                                            \
            if (first_failure_line == 0U)                                                          \
                first_failure_line = __LINE__;                                                     \
        }                                                                                          \
    } while (0)
bool VisionLink_GetLatest(VisionLinkSample *s)
{
    *s = vision;
    return s->valid;
}
void BallActuator_GetStatus(BallActuatorStatus *s)
{
    *s = motor;
}
bool BallActuator_Enable(bool value)
{
    motor_enabled = value;
    return true;
}
bool BallActuator_Stop(void)
{
    return true;
}
void BallActuator_SetCommandSession(bool value)
{
    (void)value;
}
bool BallActuator_ZeroAtCurrentPosition(void)
{
    return true;
}
bool BallActuator_SetAbsoluteAngleDeg(float a)
{
    last_command = a;
    motor.actual_angle_deg = a;
    return true;
}
bool BallActuator_PrepareAbsolutePosition(uint16_t a, uint16_t b, float c, uint16_t d)
{
    (void)a;
    (void)b;
    (void)c;
    (void)d;
    return true;
}
static void Reset(void)
{
    memset(&vision, 0, sizeof(vision));
    memset(&motor, 0, sizeof(motor));
    vision.valid = true;
    vision.confidence_pct = 95;
    vision.stable = true;
    motor.online = true;
    motor.firmware_x = true;
    motor.foc_closed_loop = true;
    motor.actual_angle_valid = true;
    last_command = 0;
    motor_enabled = false;
    BallPositionControl_Init();
    Q3Sequence_Init();
}
static void Frame(unsigned t, float p)
{
    vision.position_cm = p;
    vision.rx_time_ms = t;
    vision.source_timestamp_ms = (uint16_t)t;
    ++vision.sequence;
    motor.last_position_ms = t;
    BallPositionControl_Task(t);
}
int main(void)
{
    BallOuterPd outer;
    BallOuterPdOutput model;
    BallStateEstimator estimator;
    float shaft;
    CHECK(PendulumGeometry_EndHeightMmToMotorDeg(4, &shaft) && fabsf(shaft - 8) < .001f);
    CHECK(PendulumGeometry_EndHeightMmToMotorDeg(9, &shaft) && fabsf(shaft - 15) < .001f);
    CHECK(PendulumGeometry_EndHeightMmToMotorDeg(-3.5f, &shaft) && fabsf(shaft + 7) < .001f);
    CHECK(!PendulumGeometry_EndHeightMmToMotorDeg(13, &shaft));
    CHECK(!PendulumGeometry_EndHeightMmToMotorDeg(NAN, &shaft));
    BallStateEstimator_Init(&estimator);
    CHECK(BallStateEstimator_Update(&estimator, 0, 0));
    CHECK(BallStateEstimator_Update(&estimator, .1f, .02f));
    CHECK(estimator.position_cm > 0 && estimator.velocity_cm_s > 0);
    CHECK(!BallStateEstimator_Update(&estimator, 8, .02f));
    CHECK(!BallStateEstimator_Update(&estimator, 8, .04f));
    CHECK(BallStateEstimator_Update(&estimator, 8, .06f));
    CHECK(fabsf(estimator.position_cm - 8) < .001f && fabsf(estimator.velocity_cm_s) < .001f);
    BallOuterPd_Init(&outer);
    CHECK(BallOuterPd_Update(&outer, 5, 0, .02f, 0, &model));
    CHECK(model.p_deg < 0 && model.motor_deg < 0 && fabsf(model.shaped_beam_deg) <= .013f);
    CHECK(model.stiction_assist && fabsf(model.motor_deg + 7.0f) < .001f);
    CHECK(BallOuterPd_Update(&outer, 5, 1.0f, .02f, -7, &model));
    CHECK(!model.stiction_assist); /* 测得运动即释放静摩擦辅助 */
    BallOuterPd_Init(&outer);
    CHECK(BallOuterPd_Update(&outer, 5, 0, .02f, 8, &model));
    CHECK(model.shaped_beam_deg > .8f); /* 反向时减速，绝不跳变 */
    BallOuterPd_Init(&outer);
    CHECK(BallOuterPd_Update(&outer, .5f, 5, .02f, 0, &model));
    CHECK(model.braking && model.d_deg > 0 && model.motor_deg > 0);
    BallOuterPd_Init(&outer);
    CHECK(BallOuterPd_Update(&outer, -.5f, -5, .02f, 0, &model));
    CHECK(model.braking && model.d_deg < 0 && model.motor_deg < 0);
    CHECK(!BallOuterPd_Update(&outer, 0, 0, 0, 0, &model));
    BallPositionControlStatus status;
    Reset();
    CHECK(!BallPositionControl_StartTargetCm(12, 0));
    CHECK(BallPositionControl_StartTargetCm(5, 0));
    Frame(10, 3);
    Frame(30, 3);
    Frame(50, 3);
    BallPositionControl_GetStatus(&status);
    CHECK(status.state == BALL_CONTROL_ACTIVE && last_command < 0);
    CHECK(status.pid_updates == 1 && status.sample_dt_ms == 20);
    BallPositionControl_Task(50);
    BallPositionControl_GetStatus(&status);
    CHECK(status.pid_updates == 1);
    CHECK(BallPositionControl_SetTargetCm(-5, 50));
    Frame(70, 3.2f);
    BallPositionControl_GetStatus(&status);
    CHECK(status.state == BALL_CONTROL_ACTIVE && status.measured_velocity_cm_s > 0);
    Frame(90, 11.1f);
    CHECK(BallPositionControl_HasFault());
    Reset();
    CHECK(BallPositionControl_StartTargetCm(0, 0));
    for (unsigned t = 10; t < 1300; t += 20)
        Frame(t, 0);
    BallPositionControl_GetStatus(&status);
    CHECK(status.target_reached);
    Frame(1310, 2);
    BallPositionControl_GetStatus(&status);
    CHECK(!status.target_reached && status.state == BALL_CONTROL_ACTIVE);
    vision.valid = false;
    motor.last_position_ms = 1600;
    BallPositionControl_Task(1600);
    CHECK(BallPositionControl_HasFault());
    CHECK(motor_enabled && fabsf(last_command) < .001f);
    Reset();
    CHECK(Q3Sequence_ZeroReference(10));
    CHECK(!Q3Sequence_Start(300)); /* 旧反馈不能证明已回零 */
    motor.last_position_ms = 300;
    CHECK(Q3Sequence_Start(300));
    Q3Sequence_Stop(310);
    CHECK(Q3Sequence_ZeroReference(320));
    /* 源 uint16 时间戳回绕不得产生巨大速度。 */
    Reset();
    CHECK(BallPositionControl_StartTargetCm(0, 65500));
    Frame(65510, 0);
    Frame(65530, 0);
    Frame(65550, .1f);
    BallPositionControl_GetStatus(&status);
    CHECK(status.sample_dt_ms == 20);
    /* 使用合成测量完成完整时序：+5 为直达，
     * -5 保持控制运行，扰动只清除到达状态但不影响时序。 */
    Reset();
    CHECK(Q3Sequence_ZeroReference(0));
    motor.last_position_ms = 200;
    CHECK(Q3Sequence_Start(200));
    for (unsigned t = 210; t < 1500; t += 20)
    {
        Frame(t, 0);
        Q3Sequence_Task(t);
    }
    Q3SequenceStatus sequence;
    Q3Sequence_GetStatus(&sequence);
    CHECK(sequence.state == Q3_SEQUENCE_TO_PLUS5);
    /* 一个合成的 4.5 cm 跳变必须通过连续三帧一致
     * 的重捕获门限后才能推进时序。 */
    Frame(1510, 4.5f);
    Q3Sequence_Task(1510);
    Frame(1530, 4.5f);
    Q3Sequence_Task(1530);
    Frame(1550, 4.5f);
    Q3Sequence_Task(1550);
    Q3Sequence_GetStatus(&sequence);
    CHECK(sequence.state == Q3_SEQUENCE_TO_MINUS5);
    for (unsigned t = 1570; t < 3500; t += 20)
    {
        Frame(t, -5);
        Q3Sequence_Task(t);
    }
    Q3Sequence_GetStatus(&sequence);
    CHECK(sequence.state == Q3_SEQUENCE_FINAL_HOLD || sequence.state == Q3_SEQUENCE_RETURN_LEVEL ||
          sequence.state == Q3_SEQUENCE_WAIT_CENTER);
    if (sequence.state == Q3_SEQUENCE_FINAL_HOLD)
    {
        CHECK(BallPositionControl_IsActive());
        Frame(3510, -3);
        Q3Sequence_Task(3510);
        CHECK(BallPositionControl_IsActive() && !BallPositionControl_IsArrived());
    }
    else
    {
        CHECK(sequence.state == Q3_SEQUENCE_WAIT_CENTER);
        CHECK(!BallPositionControl_IsActive());
        motor.last_position_ms = 3510;
        CHECK(Q3Sequence_Start(3510));
    }
    Reset();
    CHECK(BallPositionControl_StartTargetCm(5, 0));
    Frame(10, 0);
    Frame(30, 0);
    ++vision.sequence;
    vision.confidence_pct = 0;
    vision.rx_time_ms = 50;
    motor.last_position_ms = 50;
    BallPositionControl_Task(50); /* 一帧坏数据 */
    CHECK(!BallPositionControl_HasFault());
    vision.confidence_pct = 95;
    Frame(70, .1f); /* 下一帧好数据恢复 PID */
    CHECK(!BallPositionControl_HasFault());
    ++vision.sequence;
    vision.rx_time_ms = 90;
    motor.last_position_ms = 90;
    BallPositionControl_Task(90); /* 源时间戳停滞：拒绝，但不触发故障 */
    CHECK(!BallPositionControl_HasFault());
    motor.last_position_ms = 350;
    BallPositionControl_Task(350);
    BallPositionControl_GetStatus(&status);
    CHECK(status.fault == BALL_CONTROL_FAULT_VISION_STALE);
    CHECK(motor_enabled && fabsf(last_command) < .001f);
    Reset();
    CHECK(BallPositionControl_StartTargetCm(5, 0));
    Frame(10, 0);
    Frame(30, 0);
    Frame(50, 0);
    BallPositionControl_GetStatus(&status);
    unsigned before_updates = status.pid_updates;
    vision.valid = false;
    motor.last_position_ms = 70;
    BallPositionControl_Task(70);
    BallPositionControl_GetStatus(&status);
    CHECK(!BallPositionControl_HasFault() && status.pid_updates == before_updates);
    CHECK(!status.target_reached && !status.vision_fresh);
    vision.valid = true;
    Frame(270, .1f); /* 220 ms 间隔仍在宽限窗口内 */
    CHECK(!BallPositionControl_HasFault());
    return failures ? (int)first_failure_line : 0;
}

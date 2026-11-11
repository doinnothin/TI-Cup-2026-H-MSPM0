#include <math.h>
#include <string.h>
#include "app/q4/q45_vehicle_control.h"
#include "app/q4/q45_vehicle_config.h"
#include "app/q4/q5_dynamic_ball_control.h"
#include "app/q4/q5_vehicle_model.h"
#include "app/q3/pendulum_geometry.h"
#include "app/ball_actuator/ball_actuator.h"
#include "code/vision_link.h"
#include "code/licheng.h"

static VisionLinkSample vision;
static BallActuatorStatus actuator;
static float chassis_speed;
static unsigned failures, first_failure_line, checks;
unsigned failure_lines[32];
unsigned failure_lines_count;
float E_speed[2];
LichengData_t g_licheng;
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        ++checks;                                                                                  \
        if (!(x))                                                                                  \
        {                                                                                          \
            if (failure_lines_count < 32U)                                                         \
                failure_lines[failure_lines_count] = __LINE__;                                     \
            ++failure_lines_count;                                                                 \
            ++failures;                                                                            \
            if (!first_failure_line)                                                               \
                first_failure_line = __LINE__;                                                     \
        }                                                                                          \
    } while (0)
bool VisionLink_GetLatest(VisionLinkSample *s)
{
    *s = vision;
    return vision.valid;
}
void BallActuator_GetStatus(BallActuatorStatus *s)
{
    *s = actuator;
}
bool BallActuator_ClearProtection(void)
{
    return true;
}
bool BallActuator_Enable(bool e)
{
    (void)e;
    return true;
}
bool BallActuator_Stop(void)
{
    return true;
}
bool BallActuator_ZeroAtCurrentPosition(void)
{
    actuator.actual_angle_deg = 0;
    return true;
}
bool BallActuator_ReturnAbsoluteZero(void)
{
    return true;
}
void BallActuator_SetCommandSession(bool e)
{
    (void)e;
}
bool BallActuator_PrepareAbsolutePosition(uint16_t a, uint16_t d, float v, uint16_t i)
{
    (void)a;
    (void)d;
    (void)v;
    (void)i;
    return true;
}
bool BallActuator_SetAbsoluteAngleDeg(float a)
{
    actuator.actual_angle_deg = a;
    return true;
}
void Control_TraceStart(float v)
{
    chassis_speed = v;
}
void Control_SetBaseSpeed(float v)
{
    chassis_speed = v;
}
void Control_Stop(void)
{
    chassis_speed = 0.0f;
}
void Licheng_Reset(void)
{
    memset(&g_licheng, 0, sizeof(g_licheng));
}

static void Frame(unsigned t, float position)
{
    vision.rx_time_ms = t;
    vision.position_cm = position;
    ++vision.sequence;
    actuator.last_position_ms = t;
    Q45Vehicle_Task(t);
}
static void Start(Q45VehicleMode mode)
{
    Q45VehicleStatus s;
    memset(&vision, 0, sizeof(vision));
    memset(&actuator, 0, sizeof(actuator));
    vision.valid = true;
    vision.confidence_pct = 95;
    vision.rx_time_ms = 1000;
    actuator.online = true;
    actuator.options_valid = true;
    actuator.firmware_x = true;
    actuator.foc_closed_loop = true;
    actuator.actual_angle_valid = true;
    actuator.last_position_ms = 1000;
    E_speed[0] = E_speed[1] = 0;
    chassis_speed = -1;
    Q45Vehicle_Init();
    CHECK(Q45Vehicle_SelectMode(mode, 1000));
    Q45Vehicle_Task(1000);
    Q45Vehicle_Task(1100);
    Q45Vehicle_Task(1120);
    Frame(1130, 0);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_READY);
    CHECK(Q45Vehicle_StartSelected(1140));
    Frame(1150, 0);
}

int main(void)
{
    Q45VehicleStatus s;
    Q5VehicleModel m;
    Q5DynamicBallController c;
    Q5DynamicBallInput in;
    Q5DynamicBallOutput out, other;
    unsigned n;
    float previous, expected;
    Q5VehicleModel_Init(&m);
    CHECK(Q5VehicleModel_Update(&m, 100, 300, -300, 220));
    CHECK(fabsf(m.speed_mm_s - 300) < .001f);
    CHECK(fabsf(m.lateral_accel_m_s2 + .18f) <
          .0001f); /* blend=-1：FIRST_TURN_ACCEL_SIGN=-1，v^2/R=0.18 */
    CHECK(!Q5VehicleModel_Update(&m, 110, 300, -300, 220));
    CHECK(Q5VehicleModel_Update(&m, 140, 310, -310, 220));
    CHECK(fabsf(m.longitudinal_accel_m_s2 - .015f) < .0001f); /* 实际 dt 40ms */
    CHECK(Q5VehicleModel_Update(&m, 500, 0, 0, 220));
    CHECK(m.speed_mm_s == 0 && m.longitudinal_accel_m_s2 == 0);
    CHECK(Q5VehicleModel_TurnBlend(138, 300) == 0);
    CHECK(Q5VehicleModel_TurnBlend(145, 300) < 0); /* 预览 149.5 -> -0.376 */
    CHECK(Q5VehicleModel_TurnBlend(145, 300) < 1);
    CHECK(Q5VehicleModel_TurnBlend(220, 300) == -1);
    CHECK(Q5VehicleModel_TurnBlend(380, 300) == 0);
    CHECK(Q5VehicleModel_TurnBlend(520, 300) == 1);
    CHECK(Q5VehicleModel_TurnBlend(620, 300) > 0); /* 24cm 混合尾部留下 +0.014 */
    CHECK(Q5VehicleModel_TurnBlend(630, 300) == 0);
    previous = Q5VehicleModel_TurnBlend(130, 300);
    for (n = 1; n < 1000; n++)
    {
        float blend = Q5VehicleModel_TurnBlend(130 + .02f * n, 300);
        CHECK(fabsf(blend - previous) < .006f);
        previous = blend;
    }

    Q5DynamicBallControl_Init(&c);
    memset(&in, 0, sizeof(in));
    in.dt_s = .03f;
    in.parallel_accel_m_s2 = .18f;
    CHECK(Q5DynamicBallControl_Update(&c, &in, &out));
    CHECK(out.feedforward_beam_deg > 1.04f &&
          out.feedforward_beam_deg < 1.06f); /* FEEDFORWARD_BEAM_SIGN=+1 */
    /* 两个零加速度平衡项共享滚动惯量。 */
    expected = out.feedforward_beam_deg * .01745329252f;
    CHECK(fabsf(9.80665f * sinf(expected) - .18f * cosf(expected)) < .00001f);
    in.parallel_accel_m_s2 = 0;
    in.measured_position_cm = -1;
    in.error_cm = 1;
    in.velocity_cm_s = -1;
    CHECK(Q5DynamicBallControl_Update(&c, &in, &out));
    CHECK(fabsf(out.p_beam_deg + .23f) < .0001f && fabsf(out.d_beam_deg + .20f) < .0001f);
    in.measured_position_cm = .10f;
    in.error_cm = -.1f;
    in.velocity_cm_s = 0;
    CHECK(Q5DynamicBallControl_Update(&c, &in, &out));
    CHECK(fabsf(out.p_beam_deg - .01f) < .0001f && out.profile == 0);
    /* 高速进入会在进入评分带前触发制动。 */
    in.measured_position_cm = 1;
    in.error_cm = -1;
    in.velocity_cm_s = -8;
    CHECK(Q5DynamicBallControl_Update(&c, &in, &out));
    CHECK(out.profile == 2 && out.raw_beam_deg < 0);
    in.measured_position_cm = .999f;
    in.error_cm = -.999f;
    in.velocity_cm_s = 0;
    CHECK(Q5DynamicBallControl_Update(&c, &in, &out));
    in.measured_position_cm = 1.001f;
    in.error_cm = -1.001f;
    CHECK(Q5DynamicBallControl_Update(&c, &in, &other));
    CHECK(fabsf(out.raw_beam_deg - other.raw_beam_deg) < .001f);
    Q5DynamicBallControl_Init(&c);
    memset(&in, 0, sizeof(in));
    in.dt_s = .03f;
    previous = 0;
    for (n = 0; n < 700; n++)
    {
        in.measured_position_cm = (n / 70) % 2 ? -9 : 9;
        in.error_cm = -in.measured_position_cm;
        in.velocity_cm_s = (n / 35) % 2 ? -15 : 15;
        in.parallel_accel_m_s2 = (n / 50) % 2 ? -.4f : .4f;
        CHECK(Q5DynamicBallControl_Update(&c, &in, &out));
        CHECK(fabsf(out.shaped_beam_deg) <= Q45_Q5_TOTAL_BEAM_LIMIT_DEG + .0001f);
        CHECK(fabsf(out.motor_deg) <= 25);
        CHECK(fabsf(out.shaped_beam_deg - previous) <=
              Q45_Q5_BEAM_RATE_LIMIT_DEG_S * .03f + .0001f);
        previous = out.shaped_beam_deg;
        in.actual_motor_deg = out.motor_deg;
    }
    in.dt_s = 0;
    CHECK(!Q5DynamicBallControl_Update(&c, &in, &out));
    in.dt_s = .3f;
    CHECK(!Q5DynamicBallControl_Update(&c, &in, &out));
    in.dt_s = .03f;
    in.parallel_accel_m_s2 = NAN;
    CHECK(!Q5DynamicBallControl_Update(&c, &in, &out));

    /* 完整状态机回归：相机位置越过所有旧调节器门限，
     * 而名义轮速必须保持连续。 */
    Start(Q45_VEHICLE_MODE_Q5);
    previous = 0;
    for (n = 1; n <= 900; n++)
    {
        unsigned t = 1140 + n * 10;
        Q45Vehicle_Tick10msISR();
        if (n % 3 == 0)
            Frame(t, (n / 3) % 2 ? .95f : 1.05f);
        CHECK(fabsf(chassis_speed - Q5VehicleModel_TraceSpeed(n * 10)) < .001f);
        CHECK(chassis_speed >= previous && chassis_speed - previous <= .501f);
        previous = chassis_speed;
    }
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING && s.band_violation && s.max_abs_position_cm >= 1.05f);
    CHECK(s.accepted_frames > 200 && s.rejected_frames == 0);
    CHECK(s.elapsed_ms == 9000 && s.speed_governor_level == 0);
    /* IRQ 不能更新模型：只有前台任务才采样编码器。 */
    previous = s.actual_speed_mm_s;
    E_speed[0] = E_speed[1] = 300;
    Q45Vehicle_Tick10msISR();
    Q45Vehicle_GetStatus(&s);
    CHECK(s.actual_speed_mm_s == previous);
    Frame(10180, 0);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.actual_speed_mm_s > previous);

    Start(Q45_VEHICLE_MODE_Q5);
    Frame(1210, 3);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.rejected_frames == 1);
    Q45Vehicle_Task(1220);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.rejected_frames == 1);
    Frame(1240, -3);
    Frame(1270, 3);
    Frame(1300, -3);
    Frame(1330, 3);
    Frame(1360, -3);
    Frame(1390, 3);
    Frame(1420, -3);
    Frame(1450, 3);
    /* 被拒绝的帧绝不刷新已接受帧看门狗：1450 时仍为 RUNNING；
       基线 Q45_Q5_VISION_TIMEOUT_MS=600 从已接受帧 1150 起计。 */
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING);
    Q45Vehicle_Task(1760);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_VISION_LOST && s.stop_requested);
    Frame(1790, 0);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_VISION_LOST);
    Start(Q45_VEHICLE_MODE_Q5);
    Frame(1210, 11.1f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_FAULT && s.max_abs_position_cm > 11);
    Start(Q45_VEHICLE_MODE_Q5);
    /* 基线 Q45_Q5_ACTUAL_ANGLE_TIMEOUT_MS=0 会禁用 Q5 时效门限：滞后 310ms 的
       执行器报告仍继续运行；只有实际角度无效才触发故障。 */
    vision.rx_time_ms = 1210;
    ++vision.sequence;
    actuator.last_position_ms = 900;
    Q45Vehicle_Task(1210);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING);
    actuator.actual_angle_valid = false;
    vision.rx_time_ms = 1240;
    ++vision.sequence;
    Q45Vehicle_Task(1240);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_FAULT);
    Start(Q45_VEHICLE_MODE_Q5);
    g_licheng.center_cm = 623;
    Q45Vehicle_Tick10msISR();
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_FINISHED && s.route_passed);
    CHECK(!Q45Vehicle_StartSelected(1250)); /* 未做新准备不得重启 */
    Start(Q45_VEHICLE_MODE_Q4);
    Q45Vehicle_Tick10msISR();
    CHECK(fabsf(chassis_speed - .6f) < .001f);
    return failures ? first_failure_line : 0;
}

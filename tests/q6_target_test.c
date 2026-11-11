/* Q6 离线回归：目标语义、平移等价性以及
 * 未改动的绝对安全包络。复用 q5_vehicle_test.c 的 Unicorn
 * 测试框架：arm-none-eabi-gcc + tests/run_q5_test.py 打印 CHECK 计数。 */
#include <math.h>
#include <string.h>
#include "app/q4/q45_vehicle_control.h"
#include "app/q4/q45_vehicle_config.h"
#include "app/q4/q5_vehicle_model.h"
#include "app/q3/pendulum_geometry.h"
#include "app/ball_actuator/ball_actuator.h"
#include "code/vision_link.h"
#include "code/licheng.h"
#include "app/q6/q6_target.h"

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
    actuator.actual_angle_deg = 0;
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

static unsigned stamp_ms;
/* 一个新鲜视觉帧：递增的相机时间戳和序号使
 * Q6 捕获窗口的重放保护得到满足。 */
static void Feed(unsigned t, float position)
{
    vision.rx_time_ms = t;
    vision.position_cm = position;
    stamp_ms += 33U;
    vision.source_timestamp_ms = (uint16_t)stamp_ms;
    ++vision.sequence;
    actuator.last_position_ms = t;
    Q45Vehicle_Task(t);
}

/* 经 WAIT_ACTUATOR/CONFIGURE/ZERO 启动进入 READY，适用于任意模式。第
 * 四次 Task 调用执行 ZERO->READY 转换，对应 Q5 基线测试所依赖的
 * Frame(1130) 调用。 */
static void BeginMode(Q45VehicleMode mode, int use_explicit, float explicit_cm)
{
    Q45VehicleStatus s;
    memset(&vision, 0, sizeof(vision));
    memset(&actuator, 0, sizeof(actuator));
    vision.valid = true;
    vision.confidence_pct = 95;
    vision.pixel_cy = 165U;
    vision.rx_time_ms = 1000;
    actuator.online = true;
    actuator.options_valid = true;
    actuator.firmware_x = true;
    actuator.foc_closed_loop = true;
    actuator.actual_angle_valid = true;
    actuator.last_position_ms = 1000;
    E_speed[0] = E_speed[1] = 0;
    chassis_speed = -1;
    stamp_ms = 500;
    Q45Vehicle_Init();
    if (use_explicit)
        CHECK(Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, explicit_cm));
    CHECK(Q45Vehicle_SelectMode(mode, 1000));
    Q45Vehicle_Task(1000);
    Q45Vehicle_Task(1100);
    Q45Vehicle_Task(1120);
    Q45Vehicle_Task(1130);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_READY);
    CHECK(!s.target_locked && s.target_cm == 0.0f);
}

/* READY 状态捕获：16 个新鲜帧 @40ms 满足 >=15 帧且
 * >=500ms 跨度。start_t 在同一条腿的各次捕获之间必须向前推进。 */
static void Capture6(unsigned start_t, float place_cm)
{
    int i;
    for (i = 0; i < 16; i++)
        Feed(start_t + i * 40, place_cm);
}

/* 共享的 RUNNING 驱动：先在基准位置预置一帧，再以 30ms 周期送入
 * pattern+base，使平移后的各条腿看到一致的帧/dt 序列。 */
static void DrivePattern(const float *pattern, int count, float base_cm, Q45VehicleStatus *rec)
{
    int n, i;
    Feed(1810, base_cm);
    for (n = 0; n < count; n++)
    {
        for (i = 0; i < 3; i++)
            Q45Vehicle_Tick10msISR();
        Feed(1840 + n * 30, pattern[n] + base_cm);
        Q45Vehicle_GetStatus(&rec[n]);
    }
}

int main(void)
{
    CHECK(Q6Target_IsPixelRowAccepted(165U));
    CHECK(Q6Target_IsPixelRowAccepted(105U));
    CHECK(Q6Target_IsPixelRowAccepted(225U));
    CHECK(!Q6Target_IsPixelRowAccepted(104U));
    CHECK(!Q6Target_IsPixelRowAccepted(226U));

    Q45VehicleStatus s, rec_a[16], rec_b[16];
    int n;
    static const float pattern[16] = {0, 2, -2, 4, -4, 6, -6, 7, -7, 5, -5, 1, -1, 3, -3, 2};
    static const float small[16] = {
        0, 1, -1, 2, -2, 3, -3, 2, -2, 1, -1, 0.5f, -0.5f, 2.5f, -2.5f, 1.5f};

    /* T_D0：无激活模式时的目标配置校验。 */
    Q45Vehicle_Init();
    CHECK(!Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, 10.5f)); /* > 5cm */
    CHECK(!Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, NAN));
    CHECK(Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, 3.0f));
    CHECK(Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_CAPTURE, 0.0f));

    /* T_A：Q6 显式零目标与捕获零目标，在相同帧/dt 序列下必须产生
     * 完全一致的输出。Q5 有其自己的最终路径规划控制律，
     * 特意不作为等价性判据。 */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(pattern, 16, 0.0f, rec_a);
    BeginMode(Q45_VEHICLE_MODE_Q6, 0, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(pattern, 16, 0.0f, rec_b);
    for (n = 0; n < 16; n++)
    {
        CHECK(rec_b[n].control_error_cm == rec_a[n].control_error_cm);
        CHECK(rec_b[n].commanded_angle_deg == rec_a[n].commanded_angle_deg);
        CHECK(rec_b[n].control_profile == rec_a[n].control_profile);
        CHECK(rec_b[n].in_scoring_band == rec_a[n].in_scoring_band);
        CHECK(rec_b[n].error_cm == rec_a[n].error_cm);
        CHECK(rec_b[n].max_abs_position_cm == rec_a[n].max_abs_position_cm);
    }
    Q45Vehicle_GetStatus(&s);
    CHECK(s.target_cm == 0.0f && s.target_locked);

    /* T_B：平移等价性。Q6(EXPLICIT 5) 输入 x+5 必须与
     * Q6(EXPLICIT 0) 输入 x 在浮点容差内一致（alpha-beta 为仿射）。 */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 5.0f);
    Capture6(1160, 5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(small, 16, 5.0f, rec_a);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.target_cm == 5.0f);
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(small, 16, 0.0f, rec_b);
    for (n = 0; n < 16; n++)
    {
        CHECK(fabsf(rec_b[n].control_error_cm - rec_a[n].control_error_cm) < 1e-3f);
        CHECK(fabsf(rec_b[n].commanded_angle_deg - rec_a[n].commanded_angle_deg) < 1e-3f);
        CHECK(rec_b[n].control_profile == rec_a[n].control_profile);
        CHECK(rec_b[n].in_scoring_band == rec_a[n].in_scoring_band);
    }

    /* T_C：目标相对语义。小球停在 +5cm 目标上 -> 零误差 HOLD；
     * 原始偏差 +0.8cm 仍在评分带内，并在负目标上镜像。
     * 各步进均低于观测器每帧跳变门限。 */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 5.0f);
    Capture6(1160, 5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, 5.0f);
    Feed(1840, 5.0f);
    Feed(1870, 5.0f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.control_profile == 3U); /* BALL_OUTER_PROFILE_RETURN_HOLD */
    CHECK(fabsf(s.error_cm) < 0.05f && s.in_scoring_band);
    Feed(1900, 5.25f);
    Feed(1930, 5.5f);
    Feed(1960, 5.75f);
    Feed(1990, 5.8f);
    Feed(2020, 5.8f);
    Q45Vehicle_GetStatus(&s);
    CHECK(fabsf(s.error_cm + 0.8f) < 0.15f);
    CHECK(s.in_scoring_band && !s.band_violation);
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, -5.0f);
    Capture6(1160, -5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, -5.0f);
    Feed(1840, -5.0f);
    Feed(1870, -5.0f);
    Feed(1900, -5.25f);
    Feed(1930, -5.5f);
    Feed(1960, -5.75f);
    Feed(1990, -5.8f);
    Feed(2020, -5.8f);
    Q45Vehicle_GetStatus(&s);
    CHECK(fabsf(s.error_cm - 0.8f) < 0.15f && s.in_scoring_band);

    /* T_D：激活模式下 ConfigureTarget 的拒绝仍然生效，而 Q6
     * 不再把绝对墙边界变成终止性停车。 */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 5.0f);
    CHECK(!Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, 1.0f));
    Capture6(1160, 5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, 5.0f);
    Feed(1840, 5.0f);
    Feed(1870, 11.2f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING && !s.fault);
    CHECK(s.max_target_dev_cm > 6.0f);

    /* T_E：捕获门限。READY 且尚未捕获时拒绝启动；
     * 低置信度帧绝不打开窗口；运行开始后
     * 已锁定目标不得随小球漂移。 */
    BeginMode(Q45_VEHICLE_MODE_Q6, 0, 0);
    CHECK(!Q45Vehicle_StartSelected(1200));
    vision.confidence_pct = 50; /* 通过模式门限 (>=10)，但不满足捕获 (>=60) */
    Capture6(1160, 0.0f);
    CHECK(!Q45Vehicle_StartSelected(1900));
    vision.confidence_pct = 95;
    Capture6(1900, -3.0f);
    CHECK(Q45Vehicle_StartSelected(2560));
    Feed(2600, -3.0f);
    Feed(2630, 2.0f);
    Feed(2660, 0.5f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING && s.target_cm == -3.0f);
    CHECK(!Q45Vehicle_StartSelected(2680));
    /* 50mm 帧跳变会被观测器新息门限拒绝，因此此处保持指令
     * 合法地接近零；T_C 已证明存在非零指令。 */

    /* T_F：终止冻结。VISION_LOST 之后峰值目标偏差
     * 停止更新，执行器驻停在零。 */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, 0.0f);
    Feed(1840, 4.0f);
    Feed(1870, -4.0f);
    Feed(1900, 6.0f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.max_target_dev_cm >= 5.99f);
    Q45Vehicle_Task(2700); /* 800ms 无新鲜帧 > 600ms 看门狗 */
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_VISION_LOST && s.fault);
    {
        float frozen = s.max_target_dev_cm;
        Q45Vehicle_Task(2800);
        Q45Vehicle_Task(2900);
        Q45Vehicle_GetStatus(&s);
        CHECK(s.max_target_dev_cm == frozen);
        CHECK(actuator.actual_angle_deg == 0.0f);
    }

    /* T_G：默认 CAPTURE 源锁定驻停小球位置的中位数。 */
    BeginMode(Q45_VEHICLE_MODE_Q6, 0, 0);
    Capture6(1160, -3.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Q45Vehicle_GetStatus(&s);
    CHECK(s.target_cm == -3.0f && s.target_locked);

    return failures ? 1 : 0;
}

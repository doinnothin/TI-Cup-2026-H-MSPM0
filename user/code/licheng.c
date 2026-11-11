#include "headfile.h"

/* 根据实际轮径、减速比和编码器参数修改。 */
#define WHEEL_DIAMETER_CM 6.5f
#define PULSE_PER_ROUND 14000.0f
#define LICHENG_PI 3.1415926f
#define CM_PER_PULSE (WHEEL_DIAMETER_CM * LICHENG_PI / PULSE_PER_ROUND)

LichengData_t g_licheng;

void Licheng_Reset(void)
{
    g_licheng.left_cm = 0.0f;
    g_licheng.right_cm = 0.0f;
    g_licheng.center_cm = 0.0f;
}

void Licheng_Init(void)
{
    Licheng_Reset();
}

/* 输入本次测速周期内左右轮新增的编码器脉冲数。 */
void Licheng_Update(int32_t left_pulse, int32_t right_pulse)
{
    g_licheng.left_cm += left_pulse * CM_PER_PULSE;
    g_licheng.right_cm += right_pulse * CM_PER_PULSE;

    /* 左、右编码器正向计数符号相反，中心里程取两侧绝对增量的平均。 */
    if (left_pulse < 0)
    {
        left_pulse = -left_pulse;
    }
    if (right_pulse < 0)
    {
        right_pulse = -right_pulse;
    }
    g_licheng.center_cm += ((float)left_pulse + (float)right_pulse) * (CM_PER_PULSE * 0.5f);
}

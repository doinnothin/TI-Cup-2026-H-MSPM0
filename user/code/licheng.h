#ifndef LICHENG_H
#define LICHENG_H

#include "headfile.h"

/* 增量式底盘里程计累加器。Update 输入为一个测速采样周期内带符号的车轮
 * 脉冲增量；距离以 cm 为单位累加。当前固件中由 Encoder_SpeedCalc() 负责
 * 周期性更新。
 */

/* 左轮、右轮和车体中心累计行驶距离，单位：cm。 */
typedef struct
{
    float left_cm;
    float right_cm;
    float center_cm;
} LichengData_t;

extern LichengData_t g_licheng;

void Licheng_Init(void);
void Licheng_Reset(void);
void Licheng_Update(int32_t left_pulse, int32_t right_pulse);

#endif

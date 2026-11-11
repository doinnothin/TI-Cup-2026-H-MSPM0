#ifndef __GYRO_H__
#define __GYRO_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* UART2 惯性传感器接口。接收解析在 UART ISR 中运行；
 * Yaw() 返回角度（度），GyroZ() 返回角速度（度/秒）。标定发送
 * 辅助函数采用忙等/延时，只能从前台代码调用。
 */
struct SAngle
{
    float Yaw;
};
extern volatile struct SAngle stcAngle;

struct SGyro
{
    float wz;
};
extern volatile struct SGyro stcGyro;

void CopeSerial2Data(unsigned char ucData);
/* 读取最近一次解析到的传感器采样；为零表示初始化以来尚未收到采样。 */
float GyroZ(void);
float Yaw(void);
/* 发送阻塞式标定序列；performCaliBias() 会阻塞约 21 秒。 */
void sendCaliYawCommand(void);
void performCaliBias(void);
void gyro_send_cmd(uint8_t *data, uint32_t len);

#endif

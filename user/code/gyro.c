#include <string.h>
#include "gyro.h"
#include "uart.h"
#include "x42s.h"
#include "delay.h"
#include "ti/driverlib/m0p/dl_core.h"

volatile unsigned char uart_data = 0;

volatile struct SAngle stcAngle;
volatile struct SGyro stcGyro;

/******************************************************************************
 * 数据解析：接收 0x5A 开头的 5 字节数据帧
******************************************************************************/
void CopeSerial2Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[11];
    static unsigned char ucRxCnt = 0;

    ucRxBuffer[ucRxCnt++] = ucData;

    if (ucRxBuffer[0] != 0x5A)
    {
        ucRxCnt = 0;
        return;
    }

    if (ucRxCnt < 5)
        return;

    unsigned char sum = 0;
    if (ucRxBuffer[1] == 0xAA)
    {
        sum = ucRxBuffer[0] + ucRxBuffer[1] + ucRxBuffer[2] + ucRxBuffer[3];

        if (sum != ucRxBuffer[4])
        {
            ucRxCnt = 0;
            return;
        }

        short wz = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
        stcGyro.wz = (float)wz / 32768.0f * 2000.0f;
    }
    else if (ucRxBuffer[1] == 0xBB)
    {
        sum = ucRxBuffer[0] + ucRxBuffer[1] + ucRxBuffer[2] + ucRxBuffer[3];

        if (sum != ucRxBuffer[4])
        {
            ucRxCnt = 0;
            return;
        }

        short rawYaw = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
        stcAngle.Yaw = (float)rawYaw / 32768.0f * 180.0f;
    }
    ucRxCnt = 0;
}

/* GYRO 接收中断（陀螺仪数据） */
void GYRO_INST_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(GYRO_INST))
    {
        CopeSerial2Data(DL_UART_Main_receiveData(GYRO_INST));
    }
}

float GyroZ(void)
{
    return stcGyro.wz;
}

float Yaw(void)
{
    return stcAngle.Yaw;
}

/* 发送命令到陀螺仪 */
void gyro_send_cmd(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        while (DL_UART_isBusy(GYRO_INST))
            ;
        DL_UART_Main_transmitData(GYRO_INST, data[i]);
    }
}

uint8_t Key[5] = {0x55, 0xAA, 0x13, 0x8E, 0x5F};
uint8_t Yaw_Zero[5] = {0x55, 0xAA, 0x15, 0x00, 0x00};
uint8_t Save[5] = {0x55, 0xAA, 0x00, 0x00, 0x00};
uint8_t BIAS_CAL[5] = {0x55, 0xAA, 0x0A, 0x01, 0x00};

void sendCaliYawCommand(void)
{
    gyro_send_cmd(Key, 5);
    delay_ms(100);
    gyro_send_cmd(Yaw_Zero, 5);
    delay_ms(100);
    gyro_send_cmd(Save, 5);
}

void performCaliBias(void)
{
    gyro_send_cmd(Key, 5);
    delay_ms(100);
    gyro_send_cmd(BIAS_CAL, 5);
    delay_ms(21000);
    gyro_send_cmd(Save, 5);
}

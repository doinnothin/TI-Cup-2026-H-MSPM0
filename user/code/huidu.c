#include "headfile.h"

uint8_t Huidu_Read(void)
{
    uint8_t value = 0;

    if (!DL_GPIO_readPins(HUIDU_S1_PORT, HUIDU_S1_PIN))
        value |= 1U << 0;
    if (!DL_GPIO_readPins(HUIDU_S2_PORT, HUIDU_S2_PIN))
        value |= 1U << 1;
    if (!DL_GPIO_readPins(HUIDU_S3_PORT, HUIDU_S3_PIN))
        value |= 1U << 2;
    if (!DL_GPIO_readPins(HUIDU_S4_PORT, HUIDU_S4_PIN))
        value |= 1U << 3;
    if (!DL_GPIO_readPins(HUIDU_S5_PORT, HUIDU_S5_PIN))
        value |= 1U << 4;
    if (!DL_GPIO_readPins(HUIDU_S6_PORT, HUIDU_S6_PIN))
        value |= 1U << 5;
    if (!DL_GPIO_readPins(HUIDU_S7_PORT, HUIDU_S7_PIN))
        value |= 1U << 6;
    if (!DL_GPIO_readPins(HUIDU_S8_PORT, HUIDU_S8_PIN))
        value |= 1U << 7;

    return value;
}

/* 约定 S1 位于车头右侧、S8 位于左侧；实车顺序相反时应反转本表。 */
static const float s_huidu_weights[8] = {-10.0f, -7.5f, -5.0f, -2.5f, 2.5f, 5.0f, 7.5f, 10.0f};

static float s_last_error;

uint8_t Huidu_IsCentered(uint8_t sensor_data)
{
    /* 较窄黑线可能只被任一中心探头看到，而居中的较宽黑线会被两个都看到。
     * 以上三种情况都视为直线。 */
    return (sensor_data == 0x08U) || /* S4 */
           (sensor_data == 0x10U) || /* S5 */
           (sensor_data == 0x18U);   /* S4 + S5 */
}

float Huidu_GetError(uint8_t sensor_data)
{
    float sum = 0.0f;
    uint8_t count = 0;

    if (sensor_data == 0xFFU)
        return 0.0f;
    if (sensor_data == 0x00U)
        return s_last_error;
    if (Huidu_IsCentered(sensor_data) != 0U)
    {
        s_last_error = 0.0f;
        return 0.0f;
    }

    for (uint8_t i = 0; i < 8; ++i)
    {
        if (sensor_data & (1U << i))
        {
            sum += s_huidu_weights[i];
            ++count;
        }
    }

    /* 单个探头压线也必须更新方向，否则窄线时会沿用旧误差。 */
    if (count != 0U)
        s_last_error = sum / (float)count;

    return s_last_error;
}

HuiduState_t Huidu_GetState(uint8_t sensor_data)
{
    if (sensor_data == 0x00U)
        return HUIDU_ALL_WHITE;
    if (sensor_data == 0xFFU)
        return HUIDU_ALL_BLACK;
    return HUIDU_TRACKING;
}

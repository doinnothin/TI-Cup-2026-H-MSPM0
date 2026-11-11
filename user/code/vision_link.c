#include "vision_link.h"
#include "bsp/app_clock.h"

#include <string.h>

#include "ti_msp_dl_config.h"

typedef enum
{
    VISION_RX_WAIT_AA = 0,
    VISION_RX_WAIT_55,
    VISION_RX_VERSION,
    VISION_RX_TYPE,
    VISION_RX_SEQUENCE,
    VISION_RX_LENGTH,
    VISION_RX_PAYLOAD,
    VISION_RX_CRC_LO,
    VISION_RX_CRC_HI
} VisionRxState;

static VisionRxState s_rx_state;
static uint8_t s_frame[6U + VISION_LINK_MAX_PAYLOAD_LEN + 2U];
static uint8_t s_frame_length;
static uint8_t s_payload_length;
static uint8_t s_payload_index;
static uint16_t s_crc_received;
static uint32_t s_now_ms;
static VisionLinkStats s_stats;
static VisionLinkSample s_samples[2];
static volatile uint8_t s_active_sample;

static uint16_t VisionLink_Crc16Modbus(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static uint16_t VisionLink_U16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int16_t VisionLink_I16LE(const uint8_t *data)
{
    return (int16_t)VisionLink_U16LE(data);
}

static void VisionLink_ResetFrame(void)
{
    s_rx_state = VISION_RX_WAIT_AA;
    s_frame_length = 0U;
    s_payload_length = 0U;
    s_payload_index = 0U;
    s_crc_received = 0U;
}

static void VisionLink_RecordSequence(uint8_t sequence)
{
    if (s_stats.have_sequence)
    {
        uint8_t expected = (uint8_t)(s_stats.last_sequence + 1U);
        s_stats.sequence_lost += (uint8_t)(sequence - expected);
    }
    s_stats.last_sequence = sequence;
    s_stats.have_sequence = true;
}

static void VisionLink_HandleFrame(void)
{
    uint16_t crc_calculated;
    uint8_t type = s_frame[3];
    uint8_t sequence = s_frame[4];
    const uint8_t *payload = &s_frame[6];
    uint8_t next;

    crc_calculated = VisionLink_Crc16Modbus(s_frame, s_frame_length - 2U);
    if (crc_calculated != s_crc_received)
    {
        ++s_stats.crc_errors;
        return;
    }

    ++s_stats.frames_ok;
    VisionLink_RecordSequence(sequence);

    if (type == VISION_LINK_TYPE_DETECT)
    {
        VisionLinkSample sample;

        sample.position_cm = (float)VisionLink_I16LE(&payload[0]) / 10.0f;
        sample.velocity_cm_s = (float)VisionLink_I16LE(&payload[2]) / 10.0f;
        sample.source_timestamp_ms = VisionLink_U16LE(&payload[4]);
        sample.stable = payload[6];
        sample.confidence_pct = payload[7];
        sample.pixel_cx = VisionLink_U16LE(&payload[8]);
        sample.pixel_cy = VisionLink_U16LE(&payload[10]);
        sample.sequence = sequence;
        sample.rx_time_ms = AppClock_NowMs();
        sample.valid = true;

        next = (uint8_t)(s_active_sample ^ 1U);
        s_samples[next] = sample;
        s_active_sample = next;
        ++s_stats.detect_frames;
    }
    else if (type == VISION_LINK_TYPE_HEARTBEAT)
    {
        ++s_stats.heartbeat_frames;
    }
    else if (type == VISION_LINK_TYPE_STATUS)
    {
        ++s_stats.status_frames;
        /* 相机在未检测到目标时发送状态帧。撤销旧的采样。 */
        next = (uint8_t)(s_active_sample ^ 1U);
        s_samples[next] = s_samples[s_active_sample];
        s_samples[next].valid = false;
        s_active_sample = next;
    }
}

static void VisionLink_OnByte(uint8_t byte)
{
    ++s_stats.bytes_received;

    switch (s_rx_state)
    {
    case VISION_RX_WAIT_AA:
        if (byte == 0xAAU)
        {
            s_frame[0] = byte;
            s_frame_length = 1U;
            s_rx_state = VISION_RX_WAIT_55;
        }
        break;

    case VISION_RX_WAIT_55:
        if (byte == 0x55U)
        {
            s_frame[1] = byte;
            s_frame_length = 2U;
            s_rx_state = VISION_RX_VERSION;
        }
        else if (byte == 0xAAU)
        {
            s_frame[0] = byte;
            s_frame_length = 1U;
        }
        else
        {
            VisionLink_ResetFrame();
        }
        break;

    case VISION_RX_VERSION:
        s_frame[2] = byte;
        s_frame_length = 3U;
        if (byte != VISION_LINK_VERSION)
        {
            ++s_stats.version_errors;
            VisionLink_ResetFrame();
        }
        else
        {
            s_rx_state = VISION_RX_TYPE;
        }
        break;

    case VISION_RX_TYPE:
        s_frame[3] = byte;
        s_frame_length = 4U;
        if ((byte != VISION_LINK_TYPE_HEARTBEAT) && (byte != VISION_LINK_TYPE_DETECT) &&
            (byte != VISION_LINK_TYPE_STATUS))
        {
            ++s_stats.type_errors;
            VisionLink_ResetFrame();
        }
        else
        {
            s_rx_state = VISION_RX_SEQUENCE;
        }
        break;

    case VISION_RX_SEQUENCE:
        s_frame[4] = byte;
        s_frame_length = 5U;
        s_rx_state = VISION_RX_LENGTH;
        break;

    case VISION_RX_LENGTH:
        s_payload_length = byte;
        s_frame[5] = byte;
        s_frame_length = 6U;
        if (s_payload_length > VISION_LINK_MAX_PAYLOAD_LEN)
        {
            ++s_stats.length_errors;
            VisionLink_ResetFrame();
        }
        else if (((s_frame[3] == VISION_LINK_TYPE_HEARTBEAT) && (s_payload_length != 1U)) ||
                 ((s_frame[3] == VISION_LINK_TYPE_DETECT) &&
                  (s_payload_length != VISION_LINK_DETECT_PAYLOAD_LEN)) ||
                 ((s_frame[3] == VISION_LINK_TYPE_STATUS) && (s_payload_length != 2U)))
        {
            ++s_stats.length_errors;
            VisionLink_ResetFrame();
        }
        else if (s_payload_length == 0U)
        {
            s_rx_state = VISION_RX_CRC_LO;
        }
        else
        {
            s_payload_index = 0U;
            s_rx_state = VISION_RX_PAYLOAD;
        }
        break;

    case VISION_RX_PAYLOAD:
        s_frame[6U + s_payload_index] = byte;
        ++s_frame_length;
        ++s_payload_index;
        if (s_payload_index >= s_payload_length)
        {
            s_rx_state = VISION_RX_CRC_LO;
        }
        break;

    case VISION_RX_CRC_LO:
        s_crc_received = byte;
        s_rx_state = VISION_RX_CRC_HI;
        break;

    case VISION_RX_CRC_HI:
        s_crc_received |= (uint16_t)byte << 8U;
        s_frame_length += 2U;
        VisionLink_HandleFrame();
        VisionLink_ResetFrame();
        break;

    default:
        VisionLink_ResetFrame();
        break;
    }
}

void VisionLink_Init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    memset(s_samples, 0, sizeof(s_samples));
    s_active_sample = 0U;
    s_now_ms = 0U;
    VisionLink_ResetFrame();
    /* 与已验证正常的接收端保持一致：不要仅依赖生成的初始化
     * 状态，因为之后的外设复位不得在无提示的情况下屏蔽 RX。 */
    DL_UART_Main_enableInterrupt(VISION_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(VISION_INST_INT_IRQN);
    NVIC_EnableIRQ(VISION_INST_INT_IRQN);
}

void VisionLink_SetNow(uint32_t now_ms)
{
    s_now_ms = now_ms;
}

void VisionLink_RxIRQ(void)
{
    ++s_stats.irq_count;
    while (!DL_UART_Main_isRXFIFOEmpty(VISION_INST))
    {
        VisionLink_OnByte(DL_UART_Main_receiveData(VISION_INST));
    }
}

void VisionLink_Poll(void)
{
    /* 解析状态与 RX ISR 共享。仅临时屏蔽 UART3 即可让此后备路径
     * 无竞争，同时不会延误其他中断。 */
    NVIC_DisableIRQ(VISION_INST_INT_IRQN);
    while (!DL_UART_Main_isRXFIFOEmpty(VISION_INST))
    {
        ++s_stats.poll_bytes;
        VisionLink_OnByte(DL_UART_Main_receiveData(VISION_INST));
    }
    NVIC_EnableIRQ(VISION_INST_INT_IRQN);
}

bool VisionLink_GetLatest(VisionLinkSample *out)
{
    uint8_t index;

    if (out == NULL)
    {
        return false;
    }

    index = s_active_sample;
    *out = s_samples[index];
    if (index != s_active_sample)
    {
        index = s_active_sample;
        *out = s_samples[index];
    }
    return out->valid;
}

void VisionLink_GetStats(VisionLinkStats *out)
{
    if (out != NULL)
    {
        *out = s_stats;
    }
}

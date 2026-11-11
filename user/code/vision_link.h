#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * UART3 视觉帧解析与最新采样接口。RxIRQ 由 UART ISR 调用，只执行
 * 有界的字节/帧记账；Poll、SetNow、GetLatest 和 GetStats 为主循环 API。
 * 采样位置/速度单位为 cm 和 cm/s，源时间和接收时间单位为 ms，置信度为百分比。
 * 仅在存在已接受的采样时 GetLatest 才返回 true；使用方在将其用于控制前
 * 仍须自行施加时效/序号策略。
 */

/* MaixCam H 题视觉协议：AA 55 01 type seq len payload crc16-modbus。 */
#define VISION_LINK_VERSION 0x01U
#define VISION_LINK_TYPE_HEARTBEAT 0x01U
#define VISION_LINK_TYPE_DETECT 0x02U
#define VISION_LINK_TYPE_STATUS 0x03U
#define VISION_LINK_DETECT_PAYLOAD_LEN 12U
#define VISION_LINK_MAX_PAYLOAD_LEN 16U

typedef struct
{
    float position_cm;
    float velocity_cm_s;
    uint16_t source_timestamp_ms;
    uint8_t stable;
    uint8_t confidence_pct;
    uint16_t pixel_cx;
    uint16_t pixel_cy;
    uint8_t sequence;
    uint32_t rx_time_ms;
    bool valid;
} VisionLinkSample;

typedef struct
{
    uint32_t bytes_received;
    uint32_t irq_count;
    uint32_t poll_bytes;
    uint32_t frames_ok;
    uint32_t detect_frames;
    uint32_t heartbeat_frames;
    uint32_t status_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
    uint32_t type_errors;
    uint32_t sequence_lost;
    uint8_t last_sequence;
    bool have_sequence;
} VisionLinkStats;

void VisionLink_Init(void);
void VisionLink_SetNow(uint32_t now_ms);
void VisionLink_RxIRQ(void);
/* 主循环后备路径。若漏掉一次 RX 中断，则排空 UART3 FIFO。 */
void VisionLink_Poll(void);
bool VisionLink_GetLatest(VisionLinkSample *out);
void VisionLink_GetStats(VisionLinkStats *out);

#endif

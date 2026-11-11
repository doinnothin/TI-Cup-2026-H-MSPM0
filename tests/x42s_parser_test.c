#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "code/x42s.h"
static const uint8_t *input;
static unsigned size, offset;
static uint8_t tx[32];
static unsigned tx_size;
bool DL_UART_Main_isRXFIFOEmpty(int instance)
{
    (void)instance;
    return offset == size;
}
uint8_t DL_UART_Main_receiveData(int instance)
{
    (void)instance;
    return input[offset++];
}
void DL_UART_Main_transmitDataBlocking(int instance, uint8_t byte)
{
    (void)instance;
    if (tx_size < sizeof(tx))
        tx[tx_size++] = byte;
}
static void Feed(const uint8_t *data, unsigned length)
{
    input = data;
    size = length;
    offset = 0;
    X42S_RxIRQ();
}
int main(void)
{
    const uint8_t options[] = {1, 0x1A, 0, 4, 0x6B};
    const uint8_t positive[] = {1, 0x36, 0, 0, 0, 0, 0x6B, 0x6B};
    const uint8_t negative[] = {1, 0x36, 1, 0, 0, 0, 0x6B, 0x6B};
    const uint8_t ack[] = {1, 0xFC, 2, 0x6B};
    X42S_Init(1);
    Feed(options, sizeof(options));
    Feed(positive, 4);
    Feed(positive + 4, 4); /* 拆分中断 / 内嵌 6B */
    if (fabsf(X42S_GetState()->position_deg - 10.7f) > .01f)
        return 1;
    Feed(negative, sizeof(negative));
    Feed(ack, sizeof(ack));
    if (fabsf(X42S_GetState()->position_deg + 10.7f) > .01f)
        return 2;
    if (X42S_GetState()->frames_received != 4 || X42S_GetState()->frame_errors)
        return 3;
    tx_size = 0;
    if (!X42S_SetQuickPositionParams(400, 700, 6.0f, X42S_POSITION_ABSOLUTE, 1200))
        return 4;
    if (tx_size != 13 || tx[0] != 1 || tx[1] != 0xF1 || tx[8] != X42S_POSITION_ABSOLUTE ||
        tx[12] != 0x6B)
        return 5;
    tx_size = 0;
    if (!X42S_SetQuickPosition(-6.5f))
        return 6;
    if (tx_size != 18 || tx[0] != 1 || tx[1] != 0xCD || tx[2] != 1 || tx[9] != 0 || tx[10] != 0 ||
        tx[11] != 0 || tx[12] != 65 || tx[13] != X42S_POSITION_ABSOLUTE || tx[17] != 0x6B)
        return 7;
    return 0;
}

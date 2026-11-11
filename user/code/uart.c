#include "uart.h"
#include "vision_link.h"
#include "x42s.h"

void uart_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

void uart_send_string(UART_Regs *uart, const char *str)
{
    while (*str != '\0')
    {
        uart_send_char(uart, (uint8_t)*str);
        str++;
    }
}

void PRINT_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(PRINT_INST))
    {
    case DL_UART_IIDX_RX:
        X42S_RxIRQ();
        break;
    default:
        break;
    }
}

void VISION_INST_IRQHandler(void)
{
    /* RX 超时和 FIFO 电平触发的 RX 都是有效的接收事件。先读取中断源
     * 将其清除，然后总是排空 FIFO；FIFO 为空也无害。 */
    (void)DL_UART_getPendingInterrupt(VISION_INST);
    VisionLink_RxIRQ();
}

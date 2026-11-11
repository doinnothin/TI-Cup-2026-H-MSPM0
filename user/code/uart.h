#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

/* 通用阻塞式 UART 发送辅助函数。只能在前台代码中使用；它们会等待
 * 每个字节发送完成，不适用于 ISR 或控制定时器。RX 分发由生成的
 * 各 UART 中断处理函数和协议模块负责。
 */
void uart_send_char(UART_Regs *uart, const uint8_t chr);
void uart_send_string(UART_Regs *uart, const char *str);

#endif

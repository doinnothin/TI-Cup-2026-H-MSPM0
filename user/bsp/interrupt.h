#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include "headfile.h"

/* 板级中断胶合层。ISR 通过下面的 volatile 字节发布按键事件；
 * 前台代码负责事件消费与应用动作。
 * 中断处理程序应保持时长有界，且不含阻塞的串口/OLED 操作。
 */
extern uint8_t enable_group1_irq;
extern volatile uint8_t g_key_t1;
extern volatile uint8_t g_key_t2;
extern volatile uint8_t g_key_t3;
extern volatile uint8_t g_key_t4;
extern volatile uint8_t g_key_start;
void Interrupt_Init(void);

#endif

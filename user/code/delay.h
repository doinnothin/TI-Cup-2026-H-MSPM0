#ifndef DELAY_H
#define DELAY_H

#include "ti_msp_dl_config.h"

/* 基于所配置 CPU 时钟的忙等延时。仅限前台使用：会阻塞调用者，
 * 不得在 ISR 或时间关键循环中使用。 */
void delay_ms(uint32_t ms);

#endif // DELAY_H

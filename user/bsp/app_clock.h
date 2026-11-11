#ifndef APP_CLOCK_H
#define APP_CLOCK_H

#include <stdint.h>

/* 由现有 TIMG6 10 ms ISR 推进的毫秒时基。从前台代码读取；
 * 无符号的经过时间相减可支持计数器回绕。 */
void AppClock_Tick10msISR(void);
uint32_t AppClock_NowMs(void);

#endif

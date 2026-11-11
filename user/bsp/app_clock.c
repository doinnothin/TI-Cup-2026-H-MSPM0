#include "app_clock.h"
static volatile uint32_t s_ms;
void AppClock_Tick10msISR(void)
{
    s_ms += 10U;
}
uint32_t AppClock_NowMs(void)
{
    return s_ms;
}

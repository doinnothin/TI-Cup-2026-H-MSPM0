#include "app_key_input.h"

#define APP_KEY_INPUT_DEBOUNCE_MS 15U

bool AppKeyInput_ConsumePress(
    GPIO_Regs *port, uint32_t pin, volatile uint8_t *flag, uint32_t now_ms, uint32_t *last_press_ms)
{
    if (*flag == 0U)
    {
        return false;
    }

    *flag = 0U;
    DL_GPIO_clearInterruptStatus(port, pin);
    if ((*last_press_ms != 0U) && ((now_ms - *last_press_ms) < APP_KEY_INPUT_DEBOUNCE_MS))
    {
        return false;
    }

    *last_press_ms = now_ms;
    return true;
}

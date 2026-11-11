#include <stdint.h>

#include "app/app_key_input.h"

static unsigned checks;
static unsigned failures;
static unsigned first_failure_line;
unsigned failure_lines[32];
unsigned failure_lines_count;
static GPIO_Regs *cleared_port;
static uint32_t cleared_pin;
static unsigned clear_count;

#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        ++checks;                                                                                  \
        if (!(condition))                                                                          \
        {                                                                                          \
            if (failure_lines_count < 32U)                                                         \
            {                                                                                      \
                failure_lines[failure_lines_count++] = __LINE__;                                   \
            }                                                                                      \
            ++failures;                                                                            \
            if (first_failure_line == 0U)                                                          \
            {                                                                                      \
                first_failure_line = __LINE__;                                                     \
            }                                                                                      \
        }                                                                                          \
    } while (0)

void DL_GPIO_clearInterruptStatus(GPIO_Regs *port, uint32_t pin)
{
    cleared_port = port;
    cleared_pin = pin;
    ++clear_count;
}

int main(void)
{
    GPIO_Regs port = {0U};
    volatile uint8_t event_flag = 0U;
    uint32_t last_press_ms = 100U;

    CHECK(!AppKeyInput_ConsumePress(&port, 0x20U, &event_flag, 105U, &last_press_ms));
    CHECK(clear_count == 0U && last_press_ms == 100U);

    event_flag = 1U;
    CHECK(!AppKeyInput_ConsumePress(&port, 0x20U, &event_flag, 110U, &last_press_ms));
    CHECK(event_flag == 0U && clear_count == 1U);
    CHECK(cleared_port == &port && cleared_pin == 0x20U && last_press_ms == 100U);

    event_flag = 1U;
    CHECK(AppKeyInput_ConsumePress(&port, 0x20U, &event_flag, 115U, &last_press_ms));
    CHECK(event_flag == 0U && last_press_ms == 115U && clear_count == 2U);

    last_press_ms = UINT32_MAX - 10U;
    event_flag = 1U;
    CHECK(AppKeyInput_ConsumePress(&port, 0x20U, &event_flag, 4U, &last_press_ms));
    CHECK(last_press_ms == 4U && clear_count == 3U);

    return failures == 0U ? 0 : 1;
}

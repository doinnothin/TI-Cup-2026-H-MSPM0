#ifndef APP_KEY_INPUT_H
#define APP_KEY_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * 前台辅助函数：消费由 ISR 记录的 GPIO 按键事件。
 * `flag` 与中断上下文共享，必须指向已有的
 * volatile 事件字节。`last_press_ms` 为调用者持有的逐键消抖状态。
 * 该函数清除挂起的 GPIO 中断状态，并应用
 * 既有的 15 ms 消抖间隔；它不决定具体应用动作。
 * 只允许在前台循环中调用，绝不能从 ISR 调用。
 */
bool AppKeyInput_ConsumePress(GPIO_Regs *port,
                              uint32_t pin,
                              volatile uint8_t *flag,
                              uint32_t now_ms,
                              uint32_t *last_press_ms);

#endif

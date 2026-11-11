#ifndef TEST_TI_CONFIG_H
#define TEST_TI_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    uint32_t test_tag;
} GPIO_Regs;
void DL_GPIO_clearInterruptStatus(GPIO_Regs *port, uint32_t pin);
#define PRINT_INST 0
#define PRINT_INST_INT_IRQN 0
bool DL_UART_Main_isRXFIFOEmpty(int instance);
uint8_t DL_UART_Main_receiveData(int instance);
void DL_UART_Main_transmitDataBlocking(int instance, uint8_t byte);
static inline void NVIC_ClearPendingIRQ(int irq)
{
    (void)irq;
}
static inline void NVIC_EnableIRQ(int irq)
{
    (void)irq;
}
static inline void NVIC_DisableIRQ(int irq)
{
    (void)irq;
}
#endif

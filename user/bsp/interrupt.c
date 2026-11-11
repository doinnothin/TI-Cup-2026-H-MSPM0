#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "encoder.h"
#include "control.h"
#include "pid.h"
#include "lap_trace.h"
#include "app_clock.h"
#include "app/q4/q45_vehicle_control.h"

uint8_t enable_group1_irq = 0;
volatile uint8_t g_key_t1 = 0;
volatile uint8_t g_key_t2 = 0;
volatile uint8_t g_key_t3 = 0;
volatile uint8_t g_key_t4 = 0;
volatile uint8_t g_key_start = 0;

void Interrupt_Init(void)
{
    if (!enable_group1_irq)
    {
        return;
    }

    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void GROUP1_IRQHandler(void)
{
    uint32_t group = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    if (group == GPIO_MULTIPLE_GPIOB_INT_IIDX)
    {
        switch (DL_GPIO_getPendingInterrupt(GPIOB))
        {
        case ENCODER_E1A_IIDX:
            DL_GPIO_clearInterruptStatus(ENCODER_E1A_PORT, ENCODER_E1A_PIN);
            Encoder_LeftA_IRQ();
            break;
        case KEY_KEY_T1_IIDX:
            DL_GPIO_clearInterruptStatus(KEY_KEY_T1_PORT, KEY_KEY_T1_PIN);
            g_key_t1 = 1;
            break;
        case KEY_KEY_T2_IIDX:
            DL_GPIO_clearInterruptStatus(KEY_KEY_T2_PORT, KEY_KEY_T2_PIN);
            g_key_t2 = 1;
            break;
        case KEY_KEY_START_IIDX:
            DL_GPIO_clearInterruptStatus(KEY_KEY_START_PORT, KEY_KEY_START_PIN);
            g_key_start = 1;
            break;
        default:
            break;
        }
    }
    else if (group == GPIO_MULTIPLE_GPIOA_INT_IIDX)
    {
        switch (DL_GPIO_getPendingInterrupt(GPIOA))
        {
        case GPIO_MULTIPLE_E2A_IIDX:
            DL_GPIO_clearInterruptStatus(GPIO_MULTIPLE_E2A_PORT, GPIO_MULTIPLE_E2A_PIN);
            Encoder_RightA_IRQ();
            break;
        case KEY_KEY_T3_IIDX:
            DL_GPIO_clearInterruptStatus(KEY_KEY_T3_PORT, KEY_KEY_T3_PIN);
            g_key_t3 = 1;
            break;
        case KEY_KEY_T4_IIDX:
            DL_GPIO_clearInterruptStatus(KEY_KEY_T4_PORT, KEY_KEY_T4_PIN);
            g_key_t4 = 1;
            break;
        default:
            break;
        }
    }
}

void Timer_Speed_INST_IRQHandler(void)
{
    Encoder_SpeedCalc();
    if (control_enable != 0U)
    {
        pid_control();
    }
}

void Timer_Control_INST_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(Timer_Control_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    AppClock_Tick10msISR();
    /* 这里 Q4/Q5 仅改变底盘速度。视觉 PD 与 X42S UART
     * 帧仍在前台任务中进行，以保持此 10 ms IRQ 时长有界。 */
    Q45Vehicle_Tick10msISR();
    Control_UpperRun();
    LapTrace_Tick10ms();
    if ((LapTrace_StopRequested() != 0U) || Q45Vehicle_StopRequested())
    {
        Control_QuickBrake();
    }
}

/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define GPIO_LFXT_PORT                                                     GPIOA
#define GPIO_LFXIN_PIN                                             DL_GPIO_PIN_3
#define GPIO_LFXIN_IOMUX                                          (IOMUX_PINCM8)
#define GPIO_LFXOUT_PIN                                            DL_GPIO_PIN_4
#define GPIO_LFXOUT_IOMUX                                         (IOMUX_PINCM9)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_LEFT */
#define PWM_LEFT_INST                                                      TIMA0
#define PWM_LEFT_INST_IRQHandler                                TIMA0_IRQHandler
#define PWM_LEFT_INST_INT_IRQN                                  (TIMA0_INT_IRQn)
#define PWM_LEFT_INST_CLK_FREQ                                           8000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_LEFT_C1_PORT                                              GPIOA
#define GPIO_PWM_LEFT_C1_PIN                                      DL_GPIO_PIN_22
#define GPIO_PWM_LEFT_C1_IOMUX                                   (IOMUX_PINCM47)
#define GPIO_PWM_LEFT_C1_IOMUX_FUNC                  IOMUX_PINCM47_PF_TIMA0_CCP1
#define GPIO_PWM_LEFT_C1_IDX                                 DL_TIMER_CC_1_INDEX

/* Defines for PWM_RIGHT */
#define PWM_RIGHT_INST                                                     TIMG7
#define PWM_RIGHT_INST_IRQHandler                               TIMG7_IRQHandler
#define PWM_RIGHT_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define PWM_RIGHT_INST_CLK_FREQ                                          8000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_RIGHT_C1_PORT                                             GPIOB
#define GPIO_PWM_RIGHT_C1_PIN                                     DL_GPIO_PIN_19
#define GPIO_PWM_RIGHT_C1_IOMUX                                  (IOMUX_PINCM45)
#define GPIO_PWM_RIGHT_C1_IOMUX_FUNC                 IOMUX_PINCM45_PF_TIMG7_CCP1
#define GPIO_PWM_RIGHT_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for Timer_Speed */
#define Timer_Speed_INST                                                 (TIMG0)
#define Timer_Speed_INST_IRQHandler                             TIMG0_IRQHandler
#define Timer_Speed_INST_INT_IRQN                               (TIMG0_INT_IRQn)
#define Timer_Speed_INST_LOAD_VALUE                                     (12499U)
/* Defines for Timer_Control */
#define Timer_Control_INST                                               (TIMG6)
#define Timer_Control_INST_IRQHandler                           TIMG6_IRQHandler
#define Timer_Control_INST_INT_IRQN                             (TIMG6_INT_IRQn)
#define Timer_Control_INST_LOAD_VALUE                                   (12499U)




/* Defines for OLED */
#define OLED_INST                                                           I2C1
#define OLED_INST_IRQHandler                                     I2C1_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C1_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 400000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                         DL_GPIO_PIN_16
#define GPIO_OLED_IOMUX_SDA                                      (IOMUX_PINCM38)
#define GPIO_OLED_IOMUX_SDA_FUNC                       IOMUX_PINCM38_PF_I2C1_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                         DL_GPIO_PIN_15
#define GPIO_OLED_IOMUX_SCL                                      (IOMUX_PINCM37)
#define GPIO_OLED_IOMUX_SCL_FUNC                       IOMUX_PINCM37_PF_I2C1_SCL


/* Defines for PRINT */
#define PRINT_INST                                                         UART0
#define PRINT_INST_FREQUENCY                                            40000000
#define PRINT_INST_IRQHandler                                   UART0_IRQHandler
#define PRINT_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_PRINT_RX_PORT                                                 GPIOA
#define GPIO_PRINT_TX_PORT                                                 GPIOA
#define GPIO_PRINT_RX_PIN                                          DL_GPIO_PIN_1
#define GPIO_PRINT_TX_PIN                                          DL_GPIO_PIN_0
#define GPIO_PRINT_IOMUX_RX                                       (IOMUX_PINCM2)
#define GPIO_PRINT_IOMUX_TX                                       (IOMUX_PINCM1)
#define GPIO_PRINT_IOMUX_RX_FUNC                        IOMUX_PINCM2_PF_UART0_RX
#define GPIO_PRINT_IOMUX_TX_FUNC                        IOMUX_PINCM1_PF_UART0_TX
#define PRINT_BAUD_RATE                                                 (115200)
#define PRINT_IBRD_40_MHZ_115200_BAUD                                       (21)
#define PRINT_FBRD_40_MHZ_115200_BAUD                                       (45)
/* Defines for DEBUG */
#define DEBUG_INST                                                         UART1
#define DEBUG_INST_FREQUENCY                                            40000000
#define DEBUG_INST_IRQHandler                                   UART1_IRQHandler
#define DEBUG_INST_INT_IRQN                                       UART1_INT_IRQn
#define GPIO_DEBUG_RX_PORT                                                 GPIOB
#define GPIO_DEBUG_TX_PORT                                                 GPIOB
#define GPIO_DEBUG_RX_PIN                                          DL_GPIO_PIN_7
#define GPIO_DEBUG_TX_PIN                                          DL_GPIO_PIN_6
#define GPIO_DEBUG_IOMUX_RX                                      (IOMUX_PINCM24)
#define GPIO_DEBUG_IOMUX_TX                                      (IOMUX_PINCM23)
#define GPIO_DEBUG_IOMUX_RX_FUNC                       IOMUX_PINCM24_PF_UART1_RX
#define GPIO_DEBUG_IOMUX_TX_FUNC                       IOMUX_PINCM23_PF_UART1_TX
#define DEBUG_BAUD_RATE                                                 (115200)
#define DEBUG_IBRD_40_MHZ_115200_BAUD                                       (21)
#define DEBUG_FBRD_40_MHZ_115200_BAUD                                       (45)
/* Defines for GYRO */
#define GYRO_INST                                                          UART2
#define GYRO_INST_FREQUENCY                                             40000000
#define GYRO_INST_IRQHandler                                    UART2_IRQHandler
#define GYRO_INST_INT_IRQN                                        UART2_INT_IRQn
#define GPIO_GYRO_RX_PORT                                                  GPIOA
#define GPIO_GYRO_TX_PORT                                                  GPIOA
#define GPIO_GYRO_RX_PIN                                          DL_GPIO_PIN_24
#define GPIO_GYRO_TX_PIN                                          DL_GPIO_PIN_21
#define GPIO_GYRO_IOMUX_RX                                       (IOMUX_PINCM54)
#define GPIO_GYRO_IOMUX_TX                                       (IOMUX_PINCM46)
#define GPIO_GYRO_IOMUX_RX_FUNC                        IOMUX_PINCM54_PF_UART2_RX
#define GPIO_GYRO_IOMUX_TX_FUNC                        IOMUX_PINCM46_PF_UART2_TX
#define GYRO_BAUD_RATE                                                  (115200)
#define GYRO_IBRD_40_MHZ_115200_BAUD                                        (21)
#define GYRO_FBRD_40_MHZ_115200_BAUD                                        (45)
/* Defines for VISION */
#define VISION_INST                                                        UART3
#define VISION_INST_FREQUENCY                                           80000000
#define VISION_INST_IRQHandler                                  UART3_IRQHandler
#define VISION_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_VISION_RX_PORT                                                GPIOA
#define GPIO_VISION_TX_PORT                                                GPIOA
#define GPIO_VISION_RX_PIN                                        DL_GPIO_PIN_25
#define GPIO_VISION_TX_PIN                                        DL_GPIO_PIN_26
#define GPIO_VISION_IOMUX_RX                                     (IOMUX_PINCM55)
#define GPIO_VISION_IOMUX_TX                                     (IOMUX_PINCM59)
#define GPIO_VISION_IOMUX_RX_FUNC                      IOMUX_PINCM55_PF_UART3_RX
#define GPIO_VISION_IOMUX_TX_FUNC                      IOMUX_PINCM59_PF_UART3_TX
#define VISION_BAUD_RATE                                                (115200)
#define VISION_IBRD_80_MHZ_115200_BAUD                                      (43)
#define VISION_FBRD_80_MHZ_115200_BAUD                                      (26)





/* Defines for LED1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define LED_LED1_PORT                                                    (GPIOB)
#define LED_LED1_PIN                                             (DL_GPIO_PIN_9)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM26)
/* Defines for LED2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define LED_LED2_PORT                                                    (GPIOA)
#define LED_LED2_PIN                                            (DL_GPIO_PIN_13)
#define LED_LED2_IOMUX                                           (IOMUX_PINCM35)
/* Defines for LED3: GPIOA.12 with pinCMx 34 on package pin 5 */
#define LED_LED3_PORT                                                    (GPIOA)
#define LED_LED3_PIN                                            (DL_GPIO_PIN_12)
#define LED_LED3_IOMUX                                           (IOMUX_PINCM34)
/* Defines for LED4: GPIOB.26 with pinCMx 57 on package pin 28 */
#define LED_LED4_PORT                                                    (GPIOB)
#define LED_LED4_PIN                                            (DL_GPIO_PIN_26)
#define LED_LED4_IOMUX                                           (IOMUX_PINCM57)
/* Defines for KEY_T1: GPIOB.11 with pinCMx 28 on package pin 63 */
#define KEY_KEY_T1_PORT                                                  (GPIOB)
// groups represented: ["ENCODER","KEY"]
// pins affected: ["E1A","KEY_T1","KEY_T2","KEY_START"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define KEY_KEY_T1_IIDX                                     (DL_GPIO_IIDX_DIO11)
#define KEY_KEY_T1_PIN                                          (DL_GPIO_PIN_11)
#define KEY_KEY_T1_IOMUX                                         (IOMUX_PINCM28)
/* Defines for KEY_T2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define KEY_KEY_T2_PORT                                                  (GPIOB)
#define KEY_KEY_T2_IIDX                                     (DL_GPIO_IIDX_DIO24)
#define KEY_KEY_T2_PIN                                          (DL_GPIO_PIN_24)
#define KEY_KEY_T2_IOMUX                                         (IOMUX_PINCM52)
/* Defines for KEY_T3: GPIOA.28 with pinCMx 3 on package pin 35 */
#define KEY_KEY_T3_PORT                                                  (GPIOA)
// groups represented: ["GPIO_MULTIPLE","KEY"]
// pins affected: ["E2A","KEY_T3","KEY_T4"]
#define GPIO_MULTIPLE_GPIOA_INT_IRQN                            (GPIOA_INT_IRQn)
#define GPIO_MULTIPLE_GPIOA_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define KEY_KEY_T3_IIDX                                     (DL_GPIO_IIDX_DIO28)
#define KEY_KEY_T3_PIN                                          (DL_GPIO_PIN_28)
#define KEY_KEY_T3_IOMUX                                          (IOMUX_PINCM3)
/* Defines for KEY_T4: GPIOA.31 with pinCMx 6 on package pin 39 */
#define KEY_KEY_T4_PORT                                                  (GPIOA)
#define KEY_KEY_T4_IIDX                                     (DL_GPIO_IIDX_DIO31)
#define KEY_KEY_T4_PIN                                          (DL_GPIO_PIN_31)
#define KEY_KEY_T4_IOMUX                                          (IOMUX_PINCM6)
/* Defines for KEY_START: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY_KEY_START_PORT                                               (GPIOB)
#define KEY_KEY_START_IIDX                                  (DL_GPIO_IIDX_DIO21)
#define KEY_KEY_START_PIN                                       (DL_GPIO_PIN_21)
#define KEY_KEY_START_IOMUX                                      (IOMUX_PINCM49)
/* Defines for AIN1: GPIOB.20 with pinCMx 48 on package pin 19 */
#define MOTOR_AIN1_PORT                                                  (GPIOB)
#define MOTOR_AIN1_PIN                                          (DL_GPIO_PIN_20)
#define MOTOR_AIN1_IOMUX                                         (IOMUX_PINCM48)
/* Defines for AIN2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define MOTOR_AIN2_PORT                                                  (GPIOA)
#define MOTOR_AIN2_PIN                                          (DL_GPIO_PIN_14)
#define MOTOR_AIN2_IOMUX                                         (IOMUX_PINCM36)
/* Defines for BIN1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define MOTOR_BIN1_PORT                                                  (GPIOA)
#define MOTOR_BIN1_PIN                                           (DL_GPIO_PIN_7)
#define MOTOR_BIN1_IOMUX                                         (IOMUX_PINCM14)
/* Defines for BIN2: GPIOA.2 with pinCMx 7 on package pin 42 */
#define MOTOR_BIN2_PORT                                                  (GPIOA)
#define MOTOR_BIN2_PIN                                           (DL_GPIO_PIN_2)
#define MOTOR_BIN2_IOMUX                                          (IOMUX_PINCM7)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOB)

/* Defines for E1A: GPIOB.18 with pinCMx 44 on package pin 15 */
#define ENCODER_E1A_IIDX                                    (DL_GPIO_IIDX_DIO18)
#define ENCODER_E1A_PIN                                         (DL_GPIO_PIN_18)
#define ENCODER_E1A_IOMUX                                        (IOMUX_PINCM44)
/* Defines for E1B: GPIOB.17 with pinCMx 43 on package pin 14 */
#define ENCODER_E1B_PIN                                         (DL_GPIO_PIN_17)
#define ENCODER_E1B_IOMUX                                        (IOMUX_PINCM43)
/* Defines for E2A: GPIOA.30 with pinCMx 5 on package pin 37 */
#define GPIO_MULTIPLE_E2A_PORT                                           (GPIOA)
#define GPIO_MULTIPLE_E2A_IIDX                              (DL_GPIO_IIDX_DIO30)
#define GPIO_MULTIPLE_E2A_PIN                                   (DL_GPIO_PIN_30)
#define GPIO_MULTIPLE_E2A_IOMUX                                   (IOMUX_PINCM5)
/* Defines for E2B: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_MULTIPLE_E2B_PORT                                           (GPIOB)
#define GPIO_MULTIPLE_E2B_PIN                                    (DL_GPIO_PIN_1)
#define GPIO_MULTIPLE_E2B_IOMUX                                  (IOMUX_PINCM13)
/* Defines for S1: GPIOB.4 with pinCMx 17 on package pin 52 */
#define HUIDU_S1_PORT                                                    (GPIOB)
#define HUIDU_S1_PIN                                             (DL_GPIO_PIN_4)
#define HUIDU_S1_IOMUX                                           (IOMUX_PINCM17)
/* Defines for S2: GPIOA.8 with pinCMx 19 on package pin 54 */
#define HUIDU_S2_PORT                                                    (GPIOA)
#define HUIDU_S2_PIN                                             (DL_GPIO_PIN_8)
#define HUIDU_S2_IOMUX                                           (IOMUX_PINCM19)
/* Defines for S3: GPIOB.5 with pinCMx 18 on package pin 53 */
#define HUIDU_S3_PORT                                                    (GPIOB)
#define HUIDU_S3_PIN                                             (DL_GPIO_PIN_5)
#define HUIDU_S3_IOMUX                                           (IOMUX_PINCM18)
/* Defines for S4: GPIOA.9 with pinCMx 20 on package pin 55 */
#define HUIDU_S4_PORT                                                    (GPIOA)
#define HUIDU_S4_PIN                                             (DL_GPIO_PIN_9)
#define HUIDU_S4_IOMUX                                           (IOMUX_PINCM20)
/* Defines for S5: GPIOA.10 with pinCMx 21 on package pin 56 */
#define HUIDU_S5_PORT                                                    (GPIOA)
#define HUIDU_S5_PIN                                            (DL_GPIO_PIN_10)
#define HUIDU_S5_IOMUX                                           (IOMUX_PINCM21)
/* Defines for S6: GPIOB.15 with pinCMx 32 on package pin 3 */
#define HUIDU_S6_PORT                                                    (GPIOB)
#define HUIDU_S6_PIN                                            (DL_GPIO_PIN_15)
#define HUIDU_S6_IOMUX                                           (IOMUX_PINCM32)
/* Defines for S7: GPIOA.11 with pinCMx 22 on package pin 57 */
#define HUIDU_S7_PORT                                                    (GPIOA)
#define HUIDU_S7_PIN                                            (DL_GPIO_PIN_11)
#define HUIDU_S7_IOMUX                                           (IOMUX_PINCM22)
/* Defines for S8: GPIOB.16 with pinCMx 33 on package pin 4 */
#define HUIDU_S8_PORT                                                    (GPIOB)
#define HUIDU_S8_PIN                                            (DL_GPIO_PIN_16)
#define HUIDU_S8_IOMUX                                           (IOMUX_PINCM33)
/* Port definition for Pin Group BOARD_LED */
#define BOARD_LED_PORT                                                   (GPIOB)

/* Defines for LED_PB22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define BOARD_LED_LED_PB22_PIN                                  (DL_GPIO_PIN_22)
#define BOARD_LED_LED_PB22_IOMUX                                 (IOMUX_PINCM50)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_LEFT_init(void);
void SYSCFG_DL_PWM_RIGHT_init(void);
void SYSCFG_DL_Timer_Speed_init(void);
void SYSCFG_DL_Timer_Control_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_PRINT_init(void);
void SYSCFG_DL_DEBUG_init(void);
void SYSCFG_DL_GYRO_init(void);
void SYSCFG_DL_VISION_init(void);



bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */

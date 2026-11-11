/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_LEFTBackup;
DL_TimerG_backupConfig gPWM_RIGHTBackup;
DL_TimerG_backupConfig gTimer_ControlBackup;
DL_UART_Main_backupConfig gVISIONBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_LEFT_init();
    SYSCFG_DL_PWM_RIGHT_init();
    SYSCFG_DL_Timer_Speed_init();
    SYSCFG_DL_Timer_Control_init();
    SYSCFG_DL_OLED_init();
    SYSCFG_DL_PRINT_init();
    SYSCFG_DL_DEBUG_init();
    SYSCFG_DL_GYRO_init();
    SYSCFG_DL_VISION_init();
    /* Ensure backup structures have no valid state */
	gPWM_LEFTBackup.backupRdy 	= false;
	gPWM_RIGHTBackup.backupRdy 	= false;
	gTimer_ControlBackup.backupRdy 	= false;
	gVISIONBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_LEFT_INST, &gPWM_LEFTBackup);
	retStatus &= DL_TimerG_saveConfiguration(PWM_RIGHT_INST, &gPWM_RIGHTBackup);
	retStatus &= DL_TimerG_saveConfiguration(Timer_Control_INST, &gTimer_ControlBackup);
	retStatus &= DL_UART_Main_saveConfiguration(VISION_INST, &gVISIONBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_LEFT_INST, &gPWM_LEFTBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(PWM_RIGHT_INST, &gPWM_RIGHTBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(Timer_Control_INST, &gTimer_ControlBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(VISION_INST, &gVISIONBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_LEFT_INST);
    DL_TimerG_reset(PWM_RIGHT_INST);
    DL_TimerG_reset(Timer_Speed_INST);
    DL_TimerG_reset(Timer_Control_INST);
    DL_I2C_reset(OLED_INST);
    DL_UART_Main_reset(PRINT_INST);
    DL_UART_Main_reset(DEBUG_INST);
    DL_UART_Main_reset(GYRO_INST);
    DL_UART_Main_reset(VISION_INST);
    DL_MathACL_reset(MATHACL);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_LEFT_INST);
    DL_TimerG_enablePower(PWM_RIGHT_INST);
    DL_TimerG_enablePower(Timer_Speed_INST);
    DL_TimerG_enablePower(Timer_Control_INST);
    DL_I2C_enablePower(OLED_INST);
    DL_UART_Main_enablePower(PRINT_INST);
    DL_UART_Main_enablePower(DEBUG_INST);
    DL_UART_Main_enablePower(GYRO_INST);
    DL_UART_Main_enablePower(VISION_INST);
    DL_MathACL_enablePower(MATHACL);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXOUT_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_LEFT_C1_IOMUX,GPIO_PWM_LEFT_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_LEFT_C1_PORT, GPIO_PWM_LEFT_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_RIGHT_C1_IOMUX,GPIO_PWM_RIGHT_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_RIGHT_C1_PORT, GPIO_PWM_RIGHT_C1_PIN);

    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_OLED_IOMUX_SDA, GPIO_OLED_IOMUX_SDA_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_OLED_IOMUX_SCL, GPIO_OLED_IOMUX_SCL_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PRINT_IOMUX_TX, GPIO_PRINT_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_PRINT_IOMUX_RX, GPIO_PRINT_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_DEBUG_IOMUX_TX, GPIO_DEBUG_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_DEBUG_IOMUX_RX, GPIO_DEBUG_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_GYRO_IOMUX_TX, GPIO_GYRO_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_GYRO_IOMUX_RX, GPIO_GYRO_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_VISION_IOMUX_TX, GPIO_VISION_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_VISION_IOMUX_RX, GPIO_VISION_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(LED_LED1_IOMUX);

    DL_GPIO_initDigitalOutput(LED_LED2_IOMUX);

    DL_GPIO_initDigitalOutput(LED_LED3_IOMUX);

    DL_GPIO_initDigitalOutput(LED_LED4_IOMUX);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY_T1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY_T2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY_T3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY_T4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY_KEY_START_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MOTOR_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_BIN2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E1A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E1B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_MULTIPLE_E2A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_MULTIPLE_E2B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(HUIDU_S8_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(BOARD_LED_LED_PB22_IOMUX);

    DL_GPIO_clearPins(GPIOA, LED_LED2_PIN |
		LED_LED3_PIN |
		MOTOR_AIN2_PIN |
		MOTOR_BIN1_PIN |
		MOTOR_BIN2_PIN);
    DL_GPIO_enableOutput(GPIOA, LED_LED2_PIN |
		LED_LED3_PIN |
		MOTOR_AIN2_PIN |
		MOTOR_BIN1_PIN |
		MOTOR_BIN2_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_28_EDGE_FALL |
		DL_GPIO_PIN_31_EDGE_FALL |
		DL_GPIO_PIN_30_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOA, KEY_KEY_T3_PIN |
		KEY_KEY_T4_PIN |
		GPIO_MULTIPLE_E2A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, KEY_KEY_T3_PIN |
		KEY_KEY_T4_PIN |
		GPIO_MULTIPLE_E2A_PIN);
    DL_GPIO_clearPins(GPIOB, LED_LED1_PIN |
		LED_LED4_PIN |
		MOTOR_AIN1_PIN |
		BOARD_LED_LED_PB22_PIN);
    DL_GPIO_enableOutput(GPIOB, LED_LED1_PIN |
		LED_LED4_PIN |
		MOTOR_AIN1_PIN |
		BOARD_LED_LED_PB22_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_11_EDGE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_24_EDGE_FALL |
		DL_GPIO_PIN_21_EDGE_FALL |
		DL_GPIO_PIN_18_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, KEY_KEY_T1_PIN |
		KEY_KEY_T2_PIN |
		KEY_KEY_START_PIN |
		ENCODER_E1A_PIN);
    DL_GPIO_enableInterrupt(GPIOB, KEY_KEY_T1_PIN |
		KEY_KEY_T2_PIN |
		KEY_KEY_START_PIN |
		ENCODER_E1A_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_HFCLK,
	.qDiv                   = 1,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_1
};

SYSCONFIG_WEAK bool SYSCFG_DL_SYSCTL_SYSPLL_init(void)
{
    bool fFCCRatioStatus = false;
    uint32_t fFCCSysoscCount;
    uint32_t fFCCPllCount;
    uint32_t fFCCRatio;
    uint32_t fccTimeOutCounter;

    DL_SYSCTL_setFCCPeriods( DL_SYSCTL_FCC_TRIG_CNT_01 );

    /* Measuring PLL. */
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
                        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK,
                        DL_SYSCTL_FCC_CLOCK_SOURCE_SYSPLLCLK2X);
    /* Get SYSPLL frequency using FCC */
    fccTimeOutCounter = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0) {
        delay_cycles(977);  /* 1x LFCLK cycle = 32MHz/32.768kHz = 977, 30.5us */
        fccTimeOutCounter++;
        if(fccTimeOutCounter > 65){
            /* Timeout set to approximately 2ms (user-customizable) */
            break;
        }
    }

    /* get measA= SYSPLLCLK2X freq wrt LFOSC*/
    fFCCPllCount = DL_SYSCTL_readFCC();

    /* Measuring SYSPLL Source */
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
                        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK,
                        DL_SYSCTL_FCC_CLOCK_SOURCE_HFCLK);
    /* Get SYSPLL frequency using FCC */
    fccTimeOutCounter = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0) {
        delay_cycles(977);  /* 1x LFCLK cycle = 32MHz/32.768kHz = 977, 30.5us */
        fccTimeOutCounter++;
        if(fccTimeOutCounter > 65){
            /* Timeout set to approximately 2ms (user-customizable) */
            break;
        }
    }

    /* get measB= SYSOSC freq wrt LFOSC*/
    fFCCSysoscCount = DL_SYSCTL_readFCC();

    /* Get ratio of both measurements*/
    fFCCRatio = (fFCCPllCount * FLOAT_TO_INT_SCALE) / fFCCSysoscCount;
    /* Check ratio is within bounds*/
    if ((FCC_LOWER_BOUND <  fFCCRatio) && (fFCCRatio < FCC_UPPER_BOUND))
    {
        /* ratio is good for proceeding into application code. */
        fFCCRatioStatus = true;
    }

    return fFCCRatioStatus;
}
static const DL_SYSCTL_LFCLKConfig gLFCLKConfig = {
    .lowCap   = false,
    .monitor  = false,
    .xt1Drive = DL_SYSCTL_LFXT_DRIVE_STRENGTH_HIGHEST,
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ,0, false);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);

    /*
     * [SYSPLL_ERR_01]
     * PLL Incorrect locking WA start.
     * Insert after every PLL enable.
     * This can lead an infinite loop if the condition persists
     * and can block entry to the application code.
     */

    while (SYSCFG_DL_SYSCTL_SYSPLL_init() == false)
    {
        /* Toggle SYSPLL enable to re-enable SYSPLL and re-check incorrect locking */
        DL_SYSCTL_disableSYSPLL();
        DL_SYSCTL_enableSYSPLL();

        /* Wait until SYSPLL startup is stabilized*/
        while ((DL_SYSCTL_getClockStatus() & SYSCTL_CLKSTATUS_SYSPLLGOOD_MASK) != DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD){}
    }
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    DL_SYSCTL_enableInterrupt((DL_SYSCTL_INTERRUPT_FLASH_SEC));

}


/*
 * Timer clock configuration to be sourced by  / 5 (16000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   8000000 Hz = 16000000 Hz / (5 * (1 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_LEFTClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_5,
    .prescale = 1U
};

static const DL_TimerA_PWMConfig gPWM_LEFTConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 1600,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_LEFT_init(void) {

    DL_TimerA_setClockConfig(
        PWM_LEFT_INST, (DL_TimerA_ClockConfig *) &gPWM_LEFTClockConfig);

    DL_TimerA_initPWMMode(
        PWM_LEFT_INST, (DL_TimerA_PWMConfig *) &gPWM_LEFTConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_LEFT_INST,DL_TIMER_CZC_CCCTL1_ZCOND,DL_TIMER_CAC_CCCTL1_ACOND,DL_TIMER_CLC_CCCTL1_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_LEFT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_LEFT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_LEFT_INST);


    
    DL_TimerA_setCCPDirection(PWM_LEFT_INST , DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 5 (16000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   8000000 Hz = 16000000 Hz / (5 * (1 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_RIGHTClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_5,
    .prescale = 1U
};

static const DL_TimerG_PWMConfig gPWM_RIGHTConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 1600,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_RIGHT_init(void) {

    DL_TimerG_setClockConfig(
        PWM_RIGHT_INST, (DL_TimerG_ClockConfig *) &gPWM_RIGHTClockConfig);

    DL_TimerG_initPWMMode(
        PWM_RIGHT_INST, (DL_TimerG_PWMConfig *) &gPWM_RIGHTConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_RIGHT_INST,DL_TIMER_CZC_CCCTL1_ZCOND,DL_TIMER_CAC_CCCTL1_ACOND,DL_TIMER_CLC_CCCTL1_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_RIGHT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_RIGHT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_RIGHT_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_RIGHT_INST);


    
    DL_TimerG_setCCPDirection(PWM_RIGHT_INST , DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   625000 Hz = 40000000 Hz / (1 * (63 + 1))
 */
static const DL_TimerG_ClockConfig gTimer_SpeedClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 63U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * Timer_Speed_INST_LOAD_VALUE = (20 ms * 625000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTimer_SpeedTimerConfig = {
    .period     = Timer_Speed_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_Timer_Speed_init(void) {

    DL_TimerG_setClockConfig(Timer_Speed_INST,
        (DL_TimerG_ClockConfig *) &gTimer_SpeedClockConfig);

    DL_TimerG_initTimerMode(Timer_Speed_INST,
        (DL_TimerG_TimerConfig *) &gTimer_SpeedTimerConfig);
    DL_TimerG_enableInterrupt(Timer_Speed_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(Timer_Speed_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1250000 Hz = 80000000 Hz / (1 * (63 + 1))
 */
static const DL_TimerG_ClockConfig gTimer_ControlClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 63U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * Timer_Control_INST_LOAD_VALUE = (10 ms * 1250000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTimer_ControlTimerConfig = {
    .period     = Timer_Control_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_Timer_Control_init(void) {

    DL_TimerG_setClockConfig(Timer_Control_INST,
        (DL_TimerG_ClockConfig *) &gTimer_ControlClockConfig);

    DL_TimerG_initTimerMode(Timer_Control_INST,
        (DL_TimerG_TimerConfig *) &gTimer_ControlTimerConfig);
    DL_TimerG_enableInterrupt(Timer_Control_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(Timer_Control_INST);





}


static const DL_I2C_ClockConfig gOLEDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_OLED_init(void) {

    DL_I2C_setClockConfig(OLED_INST,
        (DL_I2C_ClockConfig *) &gOLEDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(OLED_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(OLED_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(OLED_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(OLED_INST, 9);
    DL_I2C_setControllerTXFIFOThreshold(OLED_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(OLED_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(OLED_INST);


    /* Enable module */
    DL_I2C_enableController(OLED_INST);


}

static const DL_UART_Main_ClockConfig gPRINTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gPRINTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_PRINT_init(void)
{
    DL_UART_Main_setClockConfig(PRINT_INST, (DL_UART_Main_ClockConfig *) &gPRINTClockConfig);

    DL_UART_Main_init(PRINT_INST, (DL_UART_Main_Config *) &gPRINTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(PRINT_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(PRINT_INST, PRINT_IBRD_40_MHZ_115200_BAUD, PRINT_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(PRINT_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(PRINT_INST);
}
static const DL_UART_Main_ClockConfig gDEBUGClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gDEBUGConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_DEBUG_init(void)
{
    DL_UART_Main_setClockConfig(DEBUG_INST, (DL_UART_Main_ClockConfig *) &gDEBUGClockConfig);

    DL_UART_Main_init(DEBUG_INST, (DL_UART_Main_Config *) &gDEBUGConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(DEBUG_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(DEBUG_INST, DEBUG_IBRD_40_MHZ_115200_BAUD, DEBUG_FBRD_40_MHZ_115200_BAUD);



    DL_UART_Main_enable(DEBUG_INST);
}
static const DL_UART_Main_ClockConfig gGYROClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gGYROConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_GYRO_init(void)
{
    DL_UART_Main_setClockConfig(GYRO_INST, (DL_UART_Main_ClockConfig *) &gGYROClockConfig);

    DL_UART_Main_init(GYRO_INST, (DL_UART_Main_Config *) &gGYROConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(GYRO_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(GYRO_INST, GYRO_IBRD_40_MHZ_115200_BAUD, GYRO_FBRD_40_MHZ_115200_BAUD);



    DL_UART_Main_enable(GYRO_INST);
}
static const DL_UART_Main_ClockConfig gVISIONClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gVISIONConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_VISION_init(void)
{
    DL_UART_Main_setClockConfig(VISION_INST, (DL_UART_Main_ClockConfig *) &gVISIONClockConfig);

    DL_UART_Main_init(VISION_INST, (DL_UART_Main_Config *) &gVISIONConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(VISION_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(VISION_INST, VISION_IBRD_80_MHZ_115200_BAUD, VISION_FBRD_80_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(VISION_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(VISION_INST);
}


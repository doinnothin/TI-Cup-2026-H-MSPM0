#ifndef HEADFILE_H
#define HEADFILE_H

/* 工程公共头文件。 */

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include "ti_msp_dl_config.h"
#include <ti/driverlib/m0p/dl_interrupt.h>

#include "code/delay.h"
#include "code/uart.h"
#include "code/oled.h"
#include "code/gyro.h"
#include "code/key.h"

#include "code/licheng.h"
#include "code/encoder.h"
#include "code/motor.h"
#include "code/huidu.h"
#include "bsp/interrupt.h"
#include "bsp/pid.h"

#ifdef __cplusplus
}
#endif

#endif

#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>

/*
 * 128 x 64 OLED 绘图 API。图元/文本绘制只更新帧缓冲；
 * 需调用 OLED_Refresh() 才会传输。OLED_Clear() 会清屏并刷新。
 * 当前传输方式为阻塞式 SysConfig I2C，采用有界等待；
 * 显示函数仅限前台使用，不得在 ISR 中运行。
 * 除 API 明确采用页索引外，坐标均为像素。
 */

#define OLED_CMD 0
#define OLED_DATA 1

/* 为兼容既有调用点和字库表而保留的传统公共别名。 */
typedef unsigned char u8;
typedef unsigned int u32;

void OLED_ClearPoint(u8 x, u8 y);
void OLED_ColorTurn(u8 i);   /* 0: 正常, 1: 反色。 */
void OLED_DisplayTurn(u8 i); /* 0: 正常方向, 1: 旋转 180 度。 */

/* 为源码兼容而保留的传统软件 I2C 声明。当前硬件 I2C OLED 模块中
 * 没有对应实现。 */
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);

/* 通过当前 I2C 传输发送一个命令/数据字节。mode 为
 * OLED_CMD 或 OLED_DATA。有界的传输等待没有状态返回。 */
void OLED_WR_Byte(u8 dat, u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);

/* 帧缓冲绘图图元；调用者必须保证坐标在范围内。 */
void OLED_DrawPoint(u8 x, u8 y);
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2);
void OLED_DrawCircle(u8 x, u8 y, u8 r);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1);
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1);
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1);

/* 为源码兼容而保留的传统声明；当前 OLED 模块中没有实现。 */
void OLED_ScrollDisplay(u8 num, u8 space);

/* 页寻址直接传输：x 为列，y 为页（8 个像素行）。
 * OLED_ShowPicture 将矩形 [x0,x1) x [y0,y1) 直接写入
 * 器件，使用 BMP 中按页主序的字节；它不更新帧缓冲。
 */
void OLED_WR_BP(u8 x, u8 y);
void OLED_ShowPicture(u8 x0, u8 y0, u8 x1, u8 y1, u8 BMP[]);

/* 初始化显示屏并清屏/刷新其帧缓冲。 */
void OLED_Init(void);

#endif

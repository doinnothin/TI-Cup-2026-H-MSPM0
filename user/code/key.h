#ifndef _KEY_H_
#define _KEY_H_

#include "headfile.h"

/* 传统阻塞式按键扫描。它含延时并等待按键释放，因此不适用于
 * 当前的非阻塞主循环调度器或 ISR。 */
uint8_t Key_Scan(void);

#endif /* _KEY_H_ */

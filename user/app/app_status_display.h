#ifndef APP_STATUS_DISPLAY_H
#define APP_STATUS_DISPLAY_H

#include <stdbool.h>

#include "app/q4/q45_vehicle_control.h"

#define APP_STATUS_DISPLAY_LINE_CAPACITY 24U

/*
 * 将 Q4/Q5/Q6 状态主体渲染到已清空的 OLED 缓冲区。
 * 前提条件：vehicle 与 line_buffer 均为非空，且 line_buffer 指向
 * 至少 APP_STATUS_DISPLAY_LINE_CAPACITY 个可写字节。调用者
 * 提供最新的 vehicle 快照以及所选应用任务是否为
 * Q6。此仅前台函数负责格式化并绘制状态主体；它
 * 不清除/刷新显示，也不查询/更新控制状态。
 */
void AppStatusDisplay_ShowVehicle(const Q45VehicleStatus *vehicle, bool q6_task, char *line_buffer);

#endif

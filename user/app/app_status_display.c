#include "app_status_display.h"

#include <stdio.h>

#include "code/oled.h"

void AppStatusDisplay_ShowVehicle(const Q45VehicleStatus *vehicle, bool q6_task, char *line_buffer)
{
    int max_error10;

    OLED_ShowString(0, 16, (u8 *)Q45Vehicle_ModeName(vehicle->mode), 12);
    OLED_ShowString(32,
                    16,
                    (u8 *)((vehicle->state == Q45_VEHICLE_READY && !vehicle->center_ready)
                               ? (q6_task ? "CAPTURE" : "CENTER")
                               : Q45Vehicle_StateName(vehicle->state)),
                    12);
    (void)snprintf(line_buffer,
                   APP_STATUS_DISPLAY_LINE_CAPACITY,
                   "T:%lus D:%3dcm",
                   (unsigned long)(vehicle->elapsed_ms / 1000U),
                   (int)vehicle->route_distance_cm);
    OLED_ShowString(0, 32, (u8 *)line_buffer, 12);
    if ((vehicle->state == Q45_VEHICLE_FINISHED) || (vehicle->state == Q45_VEHICLE_TIMEOUT) ||
        (vehicle->state == Q45_VEHICLE_VISION_LOST) || (vehicle->state == Q45_VEHICLE_FAULT) ||
        (vehicle->state == Q45_VEHICLE_STOPPED))
    {
        max_error10 =
            (int)((q6_task ? vehicle->max_target_dev_cm : vehicle->max_abs_position_cm) * 10.0f +
                  0.5f);
        (void)snprintf(line_buffer,
                       APP_STATUS_DISPLAY_LINE_CAPACITY,
                       "MAX:%d.%dcm",
                       max_error10 / 10,
                       max_error10 % 10);
    }
    else if (q6_task)
    {
        /* R = 锁定的保持目标，E = 相对它的估计偏差。 */
        (void)snprintf(line_buffer,
                       APP_STATUS_DISPLAY_LINE_CAPACITY,
                       "R:%+3d E:%+3d",
                       (int)(vehicle->target_cm * 10.0f),
                       (int)(vehicle->error_cm * 10.0f));
    }
    else
    {
        (void)snprintf(line_buffer,
                       APP_STATUS_DISPLAY_LINE_CAPACITY,
                       "X:%+3d V:%+3d",
                       (int)(vehicle->measured_cm * 10.0f),
                       (int)(vehicle->measured_velocity_cm_s * 10.0f));
    }
    OLED_ShowString(0, 48, (u8 *)line_buffer, 12);
}

#pragma once
#include <stdbool.h>
/* Installation zero is explicit after mounting. A rejected zero retains the
 * previous reference and power-session trajectory; it is never a motion demo. */
void ui_v5_imu_open(void);
bool ui_v5_imu_active(void);

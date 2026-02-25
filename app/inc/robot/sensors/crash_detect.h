#ifndef CRASH_DETECT_H
#define CRASH_DETECT_H

#include <stdbool.h>

#define LSM6DSOX_WAKE_UP_SRC 0x1B
#define WAKE_UP_SRC_WU_IA (1 << 3)

typedef void (*crash_handler_t)(bool moving_forward);

int crash_detect_init(crash_handler_t handler);
void crash_detect_set_active(bool active, bool forward);
int crash_detect_deinit(void);

#endif
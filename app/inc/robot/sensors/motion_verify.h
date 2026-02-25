#ifndef MOTION_VERIFY_H
#define MOTION_VERIFY_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MOTION_STATUS_OK,          // Green: Normal operation
    MOTION_STATUS_FAST,        // Blue: Accelerometer > Encoders (downhill)
    MOTION_STATUS_SLOW,        // Yellow: Accelerometer < Encoders (uphill)
    MOTION_STATUS_CRASH        // Red: Crash detected
} motion_status_t;


int motion_verify_init(void);
void motion_verify_start(bool forward);
void motion_verify_stop(void);

motion_status_t motion_verify_get_status(void);

int motion_verify_deinit(void);

#endif // MOTION_VERIFY_H
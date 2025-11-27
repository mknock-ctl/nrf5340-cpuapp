#ifndef MOTION_VERIFY_H
#define MOTION_VERIFY_H

#include <stdbool.h>
#include <stdint.h>

#define LSM6DSOX_OUTX_L_A 0x28

typedef enum {
    MOTION_OK = 0,
    MOTION_FAST,
    MOTION_SLOW
} motion_status_t;

typedef void (*motion_status_handler_t)(motion_status_t status);

int motion_verify_init(motion_status_handler_t handler);
void motion_verify_set_active(bool active, int16_t expected_speed);
int motion_verify_deinit(void);

#endif
#ifndef TAP_DETECT_H
#define TAP_DETECT_H

#include <zephyr/kernel.h>
#include <stdbool.h>

typedef enum {
    TAP_NONE,
    TAP_SINGLE,
    TAP_DOUBLE
} tap_event_t;

int tap_detect_init(void);
bool tap_detect_wait(k_timeout_t timeout);
tap_event_t tap_detect_read_event(void);

#endif
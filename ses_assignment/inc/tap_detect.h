#ifndef TAP_DETECT_H
#define TAP_DETECT_H

#include <zephyr/kernel.h>
#include <stdbool.h>

#define TAP_SRC_SINGLE_TAP_MASK  (1 << 5)
#define TAP_SRC_DOUBLE_TAP_MASK  (1 << 4)

typedef enum {
    TAP_NONE,
    TAP_SINGLE,
    TAP_DOUBLE
} tap_event_t;

int tap_detect_init(void);
bool tap_detect_wait(k_timeout_t timeout);
void tap_detect_ignore(bool ignore);
void tap_detect_poll_test(void);

#endif
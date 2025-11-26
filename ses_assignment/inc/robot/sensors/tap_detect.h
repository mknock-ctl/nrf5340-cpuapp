#ifndef TAP_DETECT_H
#define TAP_DETECT_H

#include <stdbool.h>
#include <zephyr/kernel.h>

#define TAP_SRC_SINGLE_TAP_MASK (1 << 5)
#define TAP_SRC_DOUBLE_TAP_MASK (1 << 4)

typedef void (*tap_callback_t)(void);

int tap_detect_init(void);
bool tap_detect_wait(k_timeout_t timeout);
void tap_detect_register_callback(tap_callback_t callback);
int tap_detect_deinit(void);

#endif

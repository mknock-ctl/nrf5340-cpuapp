#ifndef HALL_H
#define HALL_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

typedef enum {
    HALL_STATE_OPEN = 0,
    HALL_STATE_CLOSED = 1,
} hall_state_t;

typedef void (*hall_cb_t)(hall_state_t state, void *user_data);

int hall_init(const struct gpio_dt_spec *gpio);

int hall_deinit(void);

hall_state_t hall_get_state(void);

int hall_register_callback(hall_cb_t cb, void *user_data);
int hall_unregister_callback(void);

#endif /* HALL_H */

#include "robot/sensors/sensor_base.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_base, LOG_LEVEL_DBG);

int sensor_base_register(sensor_base_t *s) {
    if (!s) return -EINVAL;
    LOG_DBG("Register sensor '%s'", s->name ? s->name : "<unnamed>");
    s->active = false;
    return 0;
}

int sensor_base_unregister(sensor_base_t *s) {
    if (!s) return -EINVAL;
    LOG_DBG("Unregister sensor '%s'", s->name ? s->name : "<unnamed>");
    s->active = false;
    return 0;
}

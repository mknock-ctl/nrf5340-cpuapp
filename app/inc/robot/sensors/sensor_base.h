#pragma once
#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>

struct sensor_base;

typedef struct sensor_base_ops {
    int (*init)(struct sensor_base *s);
    int (*deinit)(struct sensor_base *s);
    int (*start)(struct sensor_base *s, bool forward);
    void (*stop)(struct sensor_base *s);
    int (*irq_enable)(struct sensor_base *s, bool enable);
} sensor_base_ops_t;

typedef struct sensor_base {
    const char *name;
    const sensor_base_ops_t *ops;
    const struct gpio_dt_spec *gpio;
    bool active;
    struct gpio_int_handle *gpio_handle; /* optional, points to shared int handle if used */
    void *user_ctx;
} sensor_base_t;

int sensor_base_register(sensor_base_t *s);
int sensor_base_unregister(sensor_base_t *s);

static inline int sensor_base_start(sensor_base_t *s, bool forward) {
    if (!s || !s->ops || !s->ops->start) return -ENOTSUP;
    s->active = true;
    return s->ops->start(s, forward);
}
static inline void sensor_base_stop(sensor_base_t *s) {
    if (!s) return;
    if (s->ops && s->ops->stop) s->ops->stop(s);
    s->active = false;
}
static inline int sensor_base_init(sensor_base_t *s) {
    if (!s || !s->ops || !s->ops->init) return -ENOTSUP;
    return s->ops->init(s);
}
static inline int sensor_base_deinit(sensor_base_t *s) {
    if (!s || !s->ops || !s->ops->deinit) return -ENOTSUP;
    return s->ops->deinit(s);
}

#define SENSOR_BASE_DEFINE_OPS(_name) \
    static const sensor_base_ops_t _name##_ops

#endif /* SENSOR_BASE_H */

#include "robot/sensors/sensor_base.h"
#include "robot/sensors/hall.h"
#include "robot/sensors/int1_gpio.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hall, LOG_LEVEL_DBG);

extern struct gpio_int_handle int1_handle;

typedef struct hall_sensor {
    sensor_base_t base;

    const struct gpio_dt_spec *gpio_spec;

    volatile hall_state_t state;

    hall_cb_t user_cb;
    void *user_cb_data;
} hall_sensor_t;

static hall_sensor_t g_hall = {
    .base = {
        .name = "hall",
        .ops = NULL,
        .gpio = NULL,
        .gpio_handle = &int1_handle
    },
    .gpio_spec = NULL,
    .state = HALL_STATE_OPEN,
    .user_cb = NULL,
    .user_cb_data = NULL,
};

static int hall_init_impl(sensor_base_t *s);
static int hall_deinit_impl(sensor_base_t *s);

static void hall_gpio_isr(gpio_pin_t pin, void *user_data)
{
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);

    if (!g_hall.gpio_spec) {
        LOG_ERR("No gpio_spec for hall ISR");
        return;
    }

    int val = gpio_pin_get_dt(g_hall.gpio_spec);
    if (val < 0) {
        LOG_ERR("Failed to read hall GPIO: %d", val);
        return;
    }

    hall_state_t new_state = val ? HALL_STATE_CLOSED : HALL_STATE_OPEN;
    if (new_state == g_hall.state) {
        return;
    }

    g_hall.state = new_state;

    if (g_hall.user_cb) {
        g_hall.user_cb(new_state, g_hall.user_cb_data);
    }
}

static int hall_init_impl(sensor_base_t *s)
{
    ARG_UNUSED(s);
    if (!g_hall.gpio_spec) {
        LOG_ERR("hall_init called without gpio_spec");
        return -EINVAL;
    }

    if (!device_is_ready(g_hall.gpio_spec->port)) {
        LOG_ERR("Hall GPIO device not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(g_hall.gpio_spec, GPIO_INPUT | g_hall.gpio_spec->dt_flags);
    if (ret < 0) {
        LOG_ERR("Failed to configure hall GPIO: %d", ret);
        return ret;
    }

    int val = gpio_pin_get_dt(g_hall.gpio_spec);
    if (val < 0) {
        LOG_WRN("Unable to read initial hall pin value: %d", val);
        g_hall.state = HALL_STATE_OPEN;
    } else {
        g_hall.state = val ? HALL_STATE_CLOSED : HALL_STATE_OPEN;
    }

    ret = gpio_int_register_callback(g_hall.base.gpio_handle, hall_gpio_isr, NULL);
    if (ret < 0 && ret != -ENOMEM) {
        LOG_ERR("Failed to register hall callback: %d", ret);
        return ret;
    }

    LOG_INF("Hall initialized (state=%d)", (int)g_hall.state);
    g_hall.base.active = false;
    return 0;
}

static int hall_deinit_impl(sensor_base_t *s)
{
    ARG_UNUSED(s);

    g_hall.user_cb = NULL;
    if (g_hall.base.gpio_handle) {
        gpio_int_unregister_callback(g_hall.base.gpio_handle, hall_gpio_isr);
    }

    LOG_INF("Hall deinitialized");
    return 0;
}

SENSOR_BASE_DEFINE_OPS(hall) = {
    .init = hall_init_impl,
    .deinit = hall_deinit_impl,
    .start = NULL,
    .stop = NULL,
    .irq_enable = NULL,
};


int hall_init(const struct gpio_dt_spec *gpio)
{
    if (!gpio) {
        LOG_ERR("hall_init: gpio parameter is NULL");
        return -EINVAL;
    }

    g_hall.gpio_spec = gpio;
    g_hall.base.gpio = gpio;

    g_hall.base.ops = &hall_ops;
    sensor_base_register(&g_hall.base);
    return sensor_base_init(&g_hall.base);
}

int hall_deinit(void)
{
    sensor_base_unregister(&g_hall.base);
    g_hall.gpio_spec = NULL;
    g_hall.base.gpio = NULL;
    return sensor_base_deinit(&g_hall.base);
}

hall_state_t hall_get_state(void)
{
    return g_hall.state;
}

int hall_register_callback(hall_cb_t cb, void *user_data)
{
    if (!cb) return -EINVAL;
    g_hall.user_cb = cb;
    g_hall.user_cb_data = user_data;
    return 0;
}

int hall_unregister_callback(void)
{
    g_hall.user_cb = NULL;
    g_hall.user_cb_data = NULL;
    return 0;
}

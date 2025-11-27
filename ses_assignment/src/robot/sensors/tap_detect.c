#include "robot/sensors/sensor_base.h"
#include "robot/sensors/tap_detect.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/sensors/int1_gpio.h"  // Use the new GPIO module
#include "ses_assignment.h"
#include "robot.h"
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tap_detect, LOG_LEVEL_DBG);

extern struct gpio_int_handle int1_handle;

typedef struct tap_sensor {
    sensor_base_t base;

    struct k_sem tap_sem;
    struct k_work tap_work;
    tap_callback_t user_callback;
} tap_sensor_t;

static tap_sensor_t g_tap = {
    .base = {
        .name = "tap_detect",
        .ops = NULL, /* set in init */
        .gpio = NULL,
        .gpio_handle = &int1_handle
    },
    .user_callback = NULL,
};

static void tap_work_handler(struct k_work *work) {
    LOG_DBG("Tap work handler triggered");
    tap_sensor_t *ts = CONTAINER_OF(work, tap_sensor_t, tap_work);
    if (!ts->base.active) return;

    uint8_t tap_src = 0;
    if (lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &tap_src) != 0) {
        return;
    }

    if (tap_src & TAP_SRC_DOUBLE_TAP_MASK) {
        LOG_INF("Double Tap Detected");
        k_sem_give(&ts->tap_sem);
        if (ts->user_callback) {
            ts->user_callback();
        }
    }
}

static void int1_callback(gpio_pin_t pin, void *user_data) {
    LOG_DBG("Tap INT1 ISR triggered");
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);
    if (!g_tap.base.active) return;
    k_work_submit(&g_tap.tap_work);
    LOG_DBG("k_work_submit rc=");
}

static int tap_init(sensor_base_t *s) {
    tap_sensor_t *ts = CONTAINER_OF(s, tap_sensor_t, base);

    k_sem_init(&ts->tap_sem, 0, 1);
    k_work_init(&ts->tap_work, tap_work_handler);

    if (lsm6dsox_configure_tap_params() != 0) {
        LOG_ERR("Failed to configure tap params");
        return -EIO;
    }

    int ret = gpio_int_register_callback(ts->base.gpio_handle, int1_callback, NULL);
    if (ret < 0 && ret != -ENOMEM) {
        LOG_ERR("Register callback failed: %d", ret);
        return ret;
    }

    ret = lsm6dsox_route_int1(INT1_DOUBLE_TAP, true);
    if (ret != 0) {
        LOG_ERR("Failed to route tap interrupt: %d", ret);
        if (ret != -ENOMEM) {
            gpio_int_unregister_callback(ts->base.gpio_handle, int1_callback);
        }
        return ret;
    }

    ts->base.active = true;
    LOG_INF("Tap detection initialized");
    return 0;
}

static int tap_deinit(sensor_base_t *s) {
    tap_sensor_t *ts = CONTAINER_OF(s, tap_sensor_t, base);

    ts->user_callback = NULL;
    lsm6dsox_route_int1(INT1_DOUBLE_TAP, false);
    gpio_int_unregister_callback(ts->base.gpio_handle, int1_callback);
    LOG_INF("Tap detection deinitialized");
    return 0;
}

SENSOR_BASE_DEFINE_OPS(tap) = {
    .init = tap_init,
    .deinit = tap_deinit,
    .start = NULL,
    .stop = NULL,
    .irq_enable = NULL,
};

int tap_detect_init(void) {
    g_tap.base.ops = &tap_ops;
    sensor_base_register(&g_tap.base);
    return sensor_base_init(&g_tap.base);
}

bool tap_detect_wait(k_timeout_t timeout) {
    return k_sem_take(&g_tap.tap_sem, timeout) == 0;
}

void tap_detect_register_callback(tap_callback_t callback) {
    g_tap.user_callback = callback;
}

int tap_detect_deinit(void) {
    sensor_base_unregister(&g_tap.base);
    return sensor_base_deinit(&g_tap.base);
}
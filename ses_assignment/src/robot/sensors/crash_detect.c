#include "robot/sensors/sensor_base.h"
#include "robot/sensors/crash_detect.h"
#include "robot/sensors/int1_gpio.h"
#include "robot/sensors/lsm6dsox.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(crash_detect, LOG_LEVEL_DBG);

extern struct gpio_int_handle int1_handle;

typedef struct crash_sensor {
    sensor_base_t base;
    crash_handler_t handler;
    struct k_work crash_work;
    volatile bool moving_forward;
    int64_t activation_time;
} crash_sensor_t;

static crash_sensor_t g_crash = {
    .base = {
        .name = "crash_detect",
        .ops = NULL,
        .gpio_handle = &int1_handle
    },
    .handler = NULL,
};

static void crash_work_handler(struct k_work *work) {
    crash_sensor_t *cs = CONTAINER_OF(work, crash_sensor_t, crash_work);
    if (!cs->base.active) return;
    int64_t now = k_uptime_get();
    if ((now - cs->activation_time) < 200) {
        LOG_DBG("Debounce (elapsed %lld)", now - cs->activation_time);
        lsm6dsox_clear_interrupts();
        return;
    }
    uint8_t src;
    if (lsm6dsox_read_reg(LSM6DSOX_WAKE_UP_SRC, &src) != 0) return;
    if (!(src & WAKE_UP_SRC_WU_IA)) return;
    LOG_WRN("CRASH confirmed");
    lsm6dsox_clear_interrupts();
    if (cs->handler && cs->base.active) {
        cs->base.active = false;
        cs->handler(cs->moving_forward);
    }
}

static void crash_isr(gpio_pin_t pin, void *user_data) {
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);
    if (g_crash.base.active) {
        k_work_submit(&g_crash.crash_work);
    }
}

static int crash_init(sensor_base_t *s) {
    crash_sensor_t *cs = CONTAINER_OF(s, crash_sensor_t, base);
    if (!cs->handler) return -EINVAL;
    k_work_init(&cs->crash_work, crash_work_handler);
    lsm6dsox_clear_interrupts();
    gpio_int_register_callback(cs->base.gpio_handle, crash_isr, NULL);
    LOG_INF("Crash detection initialized");
    return 0;
}

static int crash_deinit(sensor_base_t *s) {
    crash_sensor_t *cs = CONTAINER_OF(s, crash_sensor_t, base);
    cs->base.active = false;
    gpio_int_unregister_callback(cs->base.gpio_handle, crash_isr);
    LOG_INF("Crash detection deinitialized");
    return 0;
}

static int crash_start(sensor_base_t *s, bool forward) {
    crash_sensor_t *cs = CONTAINER_OF(s, crash_sensor_t, base);
    cs->moving_forward = forward;
    cs->activation_time = k_uptime_get();
    lsm6dsox_clear_interrupts();
    LOG_DBG("Crash detection activated");
    return 0;
}

static void crash_stop(sensor_base_t *s) {
    crash_sensor_t *cs = CONTAINER_OF(s, crash_sensor_t, base);
    cs->base.active = false;
    LOG_DBG("Crash detection deactivated");
}

SENSOR_BASE_DEFINE_OPS(crash) = {
    .init = crash_init,
    .deinit = crash_deinit,
    .start = crash_start,
    .stop = crash_stop,
    .irq_enable = NULL,
};

int crash_detect_init(crash_handler_t handler) {
    g_crash.handler = handler;
    g_crash.base.ops = &crash_ops;
    sensor_base_register(&g_crash.base);
    return sensor_base_init(&g_crash.base);
}

void crash_detect_set_active(bool active, bool forward) {
    if (active) sensor_base_start(&g_crash.base, forward);
    else sensor_base_stop(&g_crash.base);
}

int crash_detect_deinit(void) {
    sensor_base_unregister(&g_crash.base);
    return sensor_base_deinit(&g_crash.base);
}
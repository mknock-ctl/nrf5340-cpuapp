#include "robot/sensors/crash_detect.h"
#include "robot/sensors/int1_gpio.h"
#include "robot/sensors/lsm6dsox.h"
#include "ses_assignment.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "robot.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(crash_detect, LOG_LEVEL_DBG);

extern struct gpio_int_handle int1_handle;

static volatile bool crash_detected = false;
static crash_handler_t handle_crash_cb = NULL;
static bool moving_fwd = false;

static struct k_work crash_work;
static bool last_was_forward = false;

static void crash_work_handler(struct k_work *work) {
    if (handle_crash_cb) {
        handle_crash_cb(last_was_forward);
    }
}

static void crash_listener(gpio_pin_t pin, void *user_data) {
    uint8_t src;
    lsm6dsox_read_reg(LSM6DSOX_WAKE_UP_SRC, &src);

    if (src & WAKE_UP_SRC_WU_IA) {
        LOG_WRN("CRASH (WakeUp) DETECTED!");
        crash_detected = true;
        last_was_forward = moving_fwd;
        k_work_submit(&crash_work);  // Defer to thread context
    }
}

int crash_detect_init(crash_handler_t handler) {
    if (!handler) return -EINVAL;
    handle_crash_cb = handler;
    k_work_init(&crash_work, crash_work_handler);

    lsm6dsox_configure_crash_params();
    gpio_int_register_callback(&int1_handle, crash_listener, NULL);
    lsm6dsox_route_int1(INT1_WU, true);

    LOG_INF("Crash detection initialized");
    return 0;
}

void crash_detect_set_active(bool active, bool forward) {
    // Soft gate, hardware interrupt still fires but we might ignore it in logic if needed
    // But since we use deinit(), we mostly rely on hardware gating.
    moving_fwd = forward;
    if(!active) crash_detected = false;
}

bool crash_detect_check(void) {
    return crash_detected;
}

int crash_detect_deinit(void) {
    lsm6dsox_route_int1(INT1_WU, false);

    TRY_ERR(int, gpio_int_unregister_callback(&int1_handle, crash_listener));
    
    LOG_INF("Crash detection deinitialized");
    return 0;
}
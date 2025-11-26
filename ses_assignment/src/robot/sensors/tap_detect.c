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

static struct k_sem tap_sem;
static struct k_work tap_work;
static tap_callback_t user_callback = NULL;

static void tap_work_handler(struct k_work *work) {
    uint8_t tap_src = 0;
    lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &tap_src);

    if (tap_src & TAP_SRC_DOUBLE_TAP_MASK) {
        LOG_INF("Double Tap Detected");
        k_sem_give(&tap_sem);
        
        if (user_callback) {
            user_callback();
        }
    }
}

static void int1_callback(gpio_pin_t pin, void *user_data) {
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);
    k_work_submit(&tap_work);
}

int tap_detect_init(void) {
    k_sem_init(&tap_sem, 0, 1);
    k_work_init(&tap_work, tap_work_handler);

    TRY_ERR(int, lsm6dsox_configure_tap_params());

    int ret = gpio_int_register_callback(&int1_handle, int1_callback, NULL);
    if (ret < 0 && ret != -ENOMEM) {
        LOG_ERR("Register callback failed: %d", ret);
        return ret;
    }

    TRY_ERR(int, lsm6dsox_route_int1(INT1_DOUBLE_TAP, true));
    
    LOG_INF("Tap detection initialized");
    return 0;
}


bool tap_detect_wait(k_timeout_t timeout) {
    return k_sem_take(&tap_sem, timeout) == 0;
}

void tap_detect_register_callback(tap_callback_t callback) {
    user_callback = callback;
}

int tap_detect_deinit(void) {
    user_callback = NULL;
    lsm6dsox_route_int1(INT1_DOUBLE_TAP, false);
    TRY_ERR(int, gpio_int_unregister_callback(&int1_handle, int1_callback));
    LOG_INF("Tap detection deinitialized");
    return 0;
}
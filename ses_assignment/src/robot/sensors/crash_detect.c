#include "robot/sensors/crash_detect.h"
#include "robot/sensors/int1_gpio.h"
#include "robot/sensors/lsm6dsox.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(crash_detect, LOG_LEVEL_DBG);

extern struct gpio_int_handle int1_handle;

static crash_handler_t crash_callback = NULL;
static volatile bool is_active = false;
static bool moving_forward = false;
static struct k_work crash_work;
static int64_t activation_time = 0;

static void crash_work_handler(struct k_work *work) {
    if (!is_active) {
        return;
    }

    int64_t now = k_uptime_get();
    if ((now - activation_time) < 200) {
        LOG_DBG("Ignoring crash - debounce period (elapsed: %lld ms)", now - activation_time);
        lsm6dsox_clear_interrupts();
        return;
    }

    uint8_t src;
    int ret = lsm6dsox_read_reg(LSM6DSOX_WAKE_UP_SRC, &src);
    
    if (ret != 0 || !(src & WAKE_UP_SRC_WU_IA)) {
        return;
    }

    LOG_WRN("CRASH confirmed");
    lsm6dsox_clear_interrupts();

    if (crash_callback && is_active) {
        is_active = false;
        crash_callback(moving_forward);
    }
}

static void crash_isr(gpio_pin_t pin, void *user_data) {
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);
    
    if (is_active) {
        k_work_submit(&crash_work);
    }
}

int crash_detect_init(crash_handler_t handler) {
    if (!handler) return -EINVAL;

    crash_callback = handler;
    is_active = false;
    k_work_init(&crash_work, crash_work_handler);
    
    lsm6dsox_configure_crash_params();
    lsm6dsox_clear_interrupts();
    
    gpio_int_register_callback(&int1_handle, crash_isr, NULL);
    lsm6dsox_route_int1(INT1_WU, true);
    
    LOG_INF("Crash detection initialized");
    return 0;
}

void crash_detect_set_active(bool active, bool forward) {
    is_active = active;
    moving_forward = forward;
    
    if (active) {
        activation_time = k_uptime_get();
        lsm6dsox_clear_interrupts();
        LOG_DBG("Crash detection activated (debounce: 200ms)");
    } else {
        LOG_DBG("Crash detection deactivated");
    }
}

int crash_detect_deinit(void) {
    is_active = false;
    lsm6dsox_route_int1(INT1_WU, false);
    gpio_int_unregister_callback(&int1_handle, crash_isr);
    LOG_INF("Crash detection deinitialized");
    return 0;
}
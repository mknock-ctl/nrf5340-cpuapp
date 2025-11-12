#include "tap_detect.h"
#include "lsm6dsox.h"
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tap_detect, LOG_LEVEL_DBG);

#define INT1_GPIO_NODE DT_NODELABEL(gpio0)
#define INT1_PIN 12

static const struct gpio_dt_spec int1_gpio = {
    .port = DEVICE_DT_GET(INT1_GPIO_NODE), .pin = INT1_PIN, .dt_flags = GPIO_ACTIVE_HIGH};

static struct gpio_callback int1_cb_data;
static struct k_sem tap_sem;
static struct k_work tap_work;

static void tap_work_handler(struct k_work *work) {
    uint8_t tap_src;

    if (lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &tap_src)) {
        return;
    }

    if (tap_src & TAP_SRC_DOUBLE_TAP) {
        LOG_INF("double tap");
        k_sem_give(&tap_sem);
    } else if (tap_src & TAP_SRC_SINGLE_TAP) {
        LOG_DBG("one tap");
    }
}

static void int1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_submit(&tap_work);
}

static int gpio_init(void) {
    if (!gpio_is_ready_dt(&int1_gpio)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT);
    if (ret) {
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_RISING);
    if (ret) {
        return ret;
    }

    gpio_init_callback(&int1_cb_data, int1_isr, BIT(int1_gpio.pin));
    gpio_add_callback(int1_gpio.port, &int1_cb_data);

    return 0;
}

int tap_detect_init(void) {
    k_sem_init(&tap_sem, 0, 1);
    k_work_init(&tap_work, tap_work_handler);

    int ret = gpio_init();
    if (ret) {
        return ret;
    }

    ret = lsm6dsox_init();
    if (ret) {
        return ret;
    }

    return 0;
}

bool tap_detect_wait(k_timeout_t timeout) { return k_sem_take(&tap_sem, timeout) == 0; }

tap_event_t tap_detect_read_event(void) {
    uint8_t tap_src;

    if (lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &tap_src)) {
        return TAP_NONE;
    }

    if (tap_src & TAP_SRC_DOUBLE_TAP) {
        return TAP_DOUBLE;
    } else if (tap_src & TAP_SRC_SINGLE_TAP) {
        return TAP_SINGLE;
    }

    return TAP_NONE;
}

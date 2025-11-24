#include "robot/sensors/tap_detect.h"
#include "robot/sensors/lsm6dsox.h"
#include "ses_assignment.h"
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
static volatile bool ignore_taps = false;

void tap_detect_ignore(bool ignore) {
    ignore_taps = ignore;
    if (ignore) {
        k_sem_reset(&tap_sem);
    }
}

static void tap_work_handler(struct k_work *work) {
    uint8_t tap_src = 0;

    TRY_ERR(int, lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &tap_src));
    LOG_DBG("TAP_SRC=0x%02X", tap_src);

    if (ignore_taps) {
        LOG_DBG("Tap ignored (ignore_taps=true)");
        return;
    }

    bool is_double = (tap_src & TAP_SRC_DOUBLE_TAP_MASK);
    bool is_single = (tap_src & TAP_SRC_SINGLE_TAP_MASK);
    LOG_DBG("TAP_SRC=0x%02X", tap_src);

    if (is_double) {
        LOG_INF(">>> DOUBLE TAP DETECTED (TAP_SRC=0x%02X) <<<", tap_src);
        k_sem_give(&tap_sem);
    } else if (is_single) {
        LOG_DBG("(Single Tap ignored, TAP_SRC=0x%02X)", tap_src);
    } else {
        LOG_DBG("Ghost Interrupt (TAP_SRC=0x%02X)", tap_src);
    }
}

static void int1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_DBG("INT1 interrupt triggered");

    k_work_submit(&tap_work);
}

static int gpio_init(void) {
    if (!gpio_is_ready_dt(&int1_gpio)) {
        return -ENODEV;
    }

    TRY_ERR(int, gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT));

    gpio_init_callback(&int1_cb_data, int1_isr, BIT(int1_gpio.pin));
    gpio_add_callback(int1_gpio.port, &int1_cb_data);

    TRY_ERR(int, gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_RISING));

    LOG_INF("GPIO INT1 configured on pin %d", int1_gpio.pin);

    return 0;
}

int tap_detect_init(void) {
    k_sem_init(&tap_sem, 0, 1);
    k_work_init(&tap_work, tap_work_handler);

    TRY_ERR(int, lsm6dsox_init());
    TRY_ERR(int, gpio_init());

    return 0;
}

bool tap_detect_wait(k_timeout_t timeout) { return k_sem_take(&tap_sem, timeout) == 0; }

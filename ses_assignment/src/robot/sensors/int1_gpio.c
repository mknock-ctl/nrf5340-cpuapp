#include "robot/sensors/int1_gpio.h"
#include <zephyr/drivers/gpio.h>
#include "ses_assignment.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_interrupt, LOG_LEVEL_DBG);

static void gpio_int_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct gpio_int_handle *handle = CONTAINER_OF(cb, struct gpio_int_handle, gpio_cb_data);

    for (int i = 0; i < GPIO_INT_MAX_CALLBACKS; i++) {
        if (handle->callbacks[i].active && handle->callbacks[i].callback) {
            handle->callbacks[i].callback(handle->gpio_spec.pin, handle->callbacks[i].user_data);
        }
    }
}

int gpio_int_init(struct gpio_int_handle *handle, const struct device *port, gpio_pin_t pin,
                  gpio_flags_t flags, gpio_flags_t int_flags, gpio_int_callback_t callback,
                  void *user_data) {
    if (!handle || !port || !callback) {
        return -EINVAL;
    }

    if (!device_is_ready(port)) {
        LOG_ERR("GPIO port not ready");
        return -ENODEV;
    }

    handle->gpio_spec.port = port;
    handle->gpio_spec.pin = pin;
    handle->gpio_spec.dt_flags = flags;
    handle->initialized = false;

    for (int i = 0; i < GPIO_INT_MAX_CALLBACKS; i++) {
        handle->callbacks[i].callback = NULL;
        handle->callbacks[i].user_data = NULL;
        handle->callbacks[i].active = false;
    }

    handle->callbacks[0].callback = callback;
    handle->callbacks[0].user_data = user_data;
    handle->callbacks[0].active = true;

    TRY_ERR(int, gpio_pin_configure(port, pin, GPIO_INPUT | flags));

    gpio_init_callback(&handle->gpio_cb_data, gpio_int_isr, BIT(pin));

    TRY_ERR(int, gpio_add_callback(port, &handle->gpio_cb_data));

    int ret = gpio_pin_interrupt_configure(port, pin, int_flags);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt on pin %d: %d", pin, ret);
        gpio_remove_callback(port, &handle->gpio_cb_data);
        return ret;
    }

    handle->initialized = true;
    LOG_INF("GPIO interrupt initialized on pin %d", pin);

    return 0;
}

int gpio_int_init_dt(struct gpio_int_handle *handle, const struct gpio_dt_spec *gpio_spec,
                     gpio_flags_t int_flags, gpio_int_callback_t callback, void *user_data) {
    if (!handle || !gpio_spec || !callback) {
        return -EINVAL;
    }

    if (!gpio_is_ready_dt(gpio_spec)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    handle->gpio_spec = *gpio_spec;
    handle->initialized = false;

    for (int i = 0; i < GPIO_INT_MAX_CALLBACKS; i++) {
        handle->callbacks[i].callback = NULL;
        handle->callbacks[i].user_data = NULL;
        handle->callbacks[i].active = false;
    }

    handle->callbacks[0].callback = callback;
    handle->callbacks[0].user_data = user_data;
    handle->callbacks[0].active = true;

    TRY_ERR(int, gpio_pin_configure_dt(gpio_spec, GPIO_INPUT));

    gpio_init_callback(&handle->gpio_cb_data, gpio_int_isr, BIT(gpio_spec->pin));

    TRY_ERR(int, gpio_add_callback(gpio_spec->port, &handle->gpio_cb_data));

    int ret = gpio_pin_interrupt_configure_dt(gpio_spec, int_flags);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt on pin %d: %d", gpio_spec->pin, ret);
        gpio_remove_callback(gpio_spec->port, &handle->gpio_cb_data);
        return ret;
    }

    handle->initialized = true;
    LOG_INF("GPIO interrupt initialized on pin %d", gpio_spec->pin);

    return 0;
}

int gpio_int_enable(struct gpio_int_handle *handle) {
    if (!handle || !handle->initialized) {
        return -EINVAL;
    }

    TRY_ERR(int, gpio_pin_interrupt_configure_dt(&handle->gpio_spec, GPIO_INT_EDGE_RISING));

    LOG_DBG("GPIO interrupt enabled on pin %d", handle->gpio_spec.pin);
    return 0;
}

int gpio_int_disable(struct gpio_int_handle *handle) {
    if (!handle || !handle->initialized) {
        return -EINVAL;
    }

    TRY_ERR(int, gpio_pin_interrupt_configure_dt(&handle->gpio_spec, GPIO_INT_DISABLE));

    LOG_DBG("GPIO interrupt disabled on pin %d", handle->gpio_spec.pin);
    return 0;
}

int gpio_int_deinit(struct gpio_int_handle *handle) {
    if (!handle || !handle->initialized) {
        return -EINVAL;
    }

    gpio_pin_interrupt_configure_dt(&handle->gpio_spec, GPIO_INT_DISABLE);

    gpio_remove_callback(handle->gpio_spec.port, &handle->gpio_cb_data);

    handle->initialized = false;
    LOG_INF("GPIO interrupt deinitialized on pin %d", handle->gpio_spec.pin);

    return 0;
}

int gpio_int_register_callback(struct gpio_int_handle *handle, gpio_int_callback_t callback,
                               void *user_data) {
    if (!handle || !handle->initialized || !callback) {
        return -EINVAL;
    }

    for (int i = 0; i < GPIO_INT_MAX_CALLBACKS; i++) {
        if (!handle->callbacks[i].active) {
            handle->callbacks[i].callback = callback;
            handle->callbacks[i].user_data = user_data;
            handle->callbacks[i].active = true;
            LOG_DBG("Registered callback #%d for pin %d", i, handle->gpio_spec.pin);
            return 0;
        }
    }

    LOG_ERR("No free callback slots for pin %d", handle->gpio_spec.pin);
    return -ENOMEM;
}

int gpio_int_unregister_callback(struct gpio_int_handle *handle, gpio_int_callback_t callback) {
    if (!handle || !handle->initialized || !callback) {
        return -EINVAL;
    }

    for (int i = 0; i < GPIO_INT_MAX_CALLBACKS; i++) {
        if (handle->callbacks[i].active && handle->callbacks[i].callback == callback) {
            handle->callbacks[i].callback = NULL;
            handle->callbacks[i].user_data = NULL;
            handle->callbacks[i].active = false;
            LOG_DBG("Unregistered callback #%d for pin %d", i, handle->gpio_spec.pin);
            return 0;
        }
    }

    LOG_WRN("Callback not found for pin %d", handle->gpio_spec.pin);
    return -ENOENT;
}

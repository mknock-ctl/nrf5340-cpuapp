#ifndef INT1_GPIO_H__
#define INT1_GPIO_H__

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define GPIO_INT_MAX_CALLBACKS 4

struct gpio_int_config {
    const struct device *port;  // GPIO port device
    gpio_pin_t pin;             // GPIO pin number
    gpio_flags_t flags;         // GPIO flags (e.g., GPIO_ACTIVE_HIGH)
    gpio_flags_t int_flags;     // Interrupt flags (e.g., GPIO_INT_EDGE_RISING)
};


typedef void (*gpio_int_callback_t)(gpio_pin_t pin, void *user_data);

struct gpio_callback_entry {
    bool active;
    gpio_int_callback_t callback;
    void *user_data;
};

struct gpio_int_handle {
    struct gpio_dt_spec gpio_spec;
    struct gpio_callback gpio_cb_data;
    struct gpio_callback_entry callbacks[GPIO_INT_MAX_CALLBACKS]; // multiple callbacks
    bool initialized;
};

int gpio_int_init(struct gpio_int_handle *handle,
                  const struct device *port,
                  gpio_pin_t pin,
                  gpio_flags_t flags,
                  gpio_flags_t int_flags,
                  gpio_int_callback_t callback,
                  void *user_data);


int gpio_int_init_dt(struct gpio_int_handle *handle,
                     const struct gpio_dt_spec *gpio_spec,
                     gpio_flags_t int_flags,
                     gpio_int_callback_t callback,
                     void *user_data);


int gpio_int_enable(struct gpio_int_handle *handle);

int gpio_int_disable(struct gpio_int_handle *handle);

int gpio_int_deinit(struct gpio_int_handle *handle);

int gpio_int_register_callback(struct gpio_int_handle *handle,
                                gpio_int_callback_t callback,
                                void *user_data);

int gpio_int_unregister_callback(struct gpio_int_handle *handle,
                                  gpio_int_callback_t callback);

#endif /* INT1_GPIO_H__ */

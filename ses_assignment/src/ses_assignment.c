/**
 * @file ses_assignment.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Stubs for SES assignment
 * @date 2025-10-01
 */

 // EIO 5		/* I/O error */
 // ENODEV 19	/* No such device */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include "ses_assignment.h"
#include <mergebot.h>

LOG_MODULE_REGISTER(ses_assignment, LOG_LEVEL_DBG);

#define I2C1_NODE DT_NODELABEL(lsm6dsox)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

static const struct gpio_dt_spec int1_gpio = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 12,
    .dt_flags = GPIO_ACTIVE_HIGH
};
static struct gpio_callback int1_cb_data;

static K_SEM_DEFINE(double_tap_sem, 0, 1);
static struct k_work tap_work;
static void tap_work_handler(struct k_work *work);

// registers 
#define LSM6DSOX_WHO_AM_I        0x0F
#define LSM6DSOX_CTRL1_XL        0x10
#define LSM6DSOX_CTRL3_C         0x12
#define LSM6DSOX_CTRL4_C         0x13
#define LSM6DSOX_TAP_CFG0        0x56
#define LSM6DSOX_TAP_CFG1        0x57
#define LSM6DSOX_TAP_CFG2        0x58
#define LSM6DSOX_TAP_THS_6D      0x59
#define LSM6DSOX_INT_DUR2        0x5A
#define LSM6DSOX_WAKE_UP_THS     0x5B
#define LSM6DSOX_WAKE_UP_DUR     0x5C
#define LSM6DSOX_MD1_CFG         0x5E
#define LSM6DSOX_TAP_SRC         0x1C
#define LSM6DSOX_ALL_INT_SRC     0x1D

#define VCAP_MAX 3000
#define VCAP_MIN 600

static int lsm6dsox_write_reg(uint8_t reg, uint8_t value) // over i2c
{
    uint8_t buf[2] = { reg, value };
    return i2c_write_dt(&dev_i2c, buf, sizeof(buf));
}

static int lsm6dsox_read_reg(uint8_t reg, uint8_t *value) // over i2c
{
    return i2c_write_read_dt(&dev_i2c, &reg, 1, value, 1);
}

static void tap_work_handler(struct k_work *work)
{
    uint8_t all_int_src = 0;
    uint8_t tap_src = 0;
    int ret;

    if (!device_is_ready(dev_i2c.bus)) {
        LOG_ERR("i2c bus = work handlmer not ready");
        return;
    }

    ret = lsm6dsox_read_reg(LSM6DSOX_ALL_INT_SRC, &all_int_src); if (ret) return ret;
    ret = lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &tap_src); if (ret) return ret;

    // idk moet nog wat verbeterd worden
    if ((all_int_src & 0x20) || (tap_src & 0x20)) {
        LOG_INF("Double-tap detected");
        k_sem_give(&double_tap_sem);
    }
}

// isr
static void int1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&tap_work);
}

static int configure_double_tap(void)
{
    int ret;
    uint8_t readback = 0;

    if (!device_is_ready(dev_i2c.bus)) {
        LOG_ERR("i2c bus not ready");
        return -ENODEV;
    }

    ret = lsm6dsox_write_reg(LSM6DSOX_CTRL1_XL, 0x60); if (ret) return ret;
    lsm6dsox_read_reg(LSM6DSOX_CTRL1_XL, &readback);
    LOG_DBG("CTRL1_XL: 0x%02X", readback);
    ret = lsm6dsox_write_reg(LSM6DSOX_CTRL3_C, 0x44); if (ret) return ret;
    ret = lsm6dsox_write_reg(LSM6DSOX_CTRL4_C, 0x00); if (ret) return ret;
    ret = lsm6dsox_write_reg(LSM6DSOX_TAP_CFG0, 0x8E); if (ret) return ret;
    lsm6dsox_read_reg(LSM6DSOX_TAP_CFG0, &readback);
    LOG_DBG("TAP_CFG0: 0x%02X", readback);
    ret = lsm6dsox_write_reg(LSM6DSOX_TAP_CFG1, 0x00); if (ret) return ret;
    ret = lsm6dsox_write_reg(LSM6DSOX_TAP_CFG2, 0x80); if (ret) return ret;
    lsm6dsox_read_reg(LSM6DSOX_TAP_CFG2, &readback);
    LOG_DBG("TAP_CFG2: 0x%02X", readback);
    ret = lsm6dsox_write_reg(LSM6DSOX_TAP_THS_6D, 0x8C); if (ret) return ret;
    lsm6dsox_read_reg(LSM6DSOX_TAP_THS_6D, &readback);
    LOG_DBG("TAP_THS_6D: 0x%02X", readback);
    ret = lsm6dsox_write_reg(LSM6DSOX_INT_DUR2, 0x7F); if (ret) return ret;
    lsm6dsox_read_reg(LSM6DSOX_INT_DUR2, &readback);
    LOG_DBG("INT_DUR2: 0x%02X", readback);
    ret = lsm6dsox_write_reg(LSM6DSOX_WAKE_UP_THS, 0x00); if (ret) return ret;
    ret = lsm6dsox_write_reg(LSM6DSOX_MD1_CFG, 0x48); if (ret) return ret;
    lsm6dsox_read_reg(LSM6DSOX_MD1_CFG, &readback);
    LOG_DBG("MD1_CFG: 0x%02X", readback);
    LOG_INF("doubletap configured!!! ");
    return 0;
}

static void update_vcap_led(int vcap)
{
    static int last_percentage = -1;
    int percentage;

    if (vcap >= VCAP_MAX) percentage = 100;
    else if (vcap <= VCAP_MIN) percentage = 0;
    else percentage = (100 * (vcap - VCAP_MIN)) / (VCAP_MAX - VCAP_MIN);

    int category = (percentage > 66) ? 2 : (percentage > 33) ? 1 : 0;
    int last_category = (last_percentage > 66) ? 2 : (last_percentage > 33) ? 1 : 0;

    if (category != last_category || last_percentage < 0) {
        mb_leds_off();
        if (percentage > 66) {
            mb_led_toggle(MB_LED_G);
        } else if (percentage > 33) {
            mb_led_toggle(MB_LED_R);
            mb_led_toggle(MB_LED_G);
        } else {
            mb_led_toggle(MB_LED_R);
        }
        last_percentage = percentage;
    }
}

static int init_int1_gpio(void)
{
    if (!gpio_is_ready_dt(&int1_gpio)) return -ENODEV;
    if (gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT)) return -EIO;
    if (gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_RISING)) return -EIO;
    gpio_init_callback(&int1_cb_data, int1_callback, BIT(int1_gpio.pin));
    gpio_add_callback(int1_gpio.port, &int1_cb_data);
    return 0;
}

void wait_for_double_tap(void)
{
    k_work_init(&tap_work, tap_work_handler);

    if (init_int1_gpio()) { LOG_ERR("INT1 init failed"); return; }
    if (configure_double_tap()) { LOG_ERR("douibletap config failed"); return; }

    LOG_INF("tap me twice quickly");
    while (k_sem_take(&double_tap_sem, K_MSEC(100)) != 0) {
        int vcap = mb_measure_vcap();
        update_vcap_led(vcap);
    }
    mb_leds_off();
    LOG_INF("doubletap received!!!!!!!!!!!!");
}

void move(int32_t distance) {
    LOG_ERR("Move not implemented");
 }
void turn(int32_t angle) { }
void turn_to_north(void) { }
void update_status(ses_status_code code) {  }

void imu_init(void)
{
    if (!device_is_ready(dev_i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", dev_i2c.bus ? dev_i2c.bus->name : "unknown");
        return;
    }
    k_work_init(&tap_work, tap_work_handler);
}

void imu_test(void)
{
    uint8_t who_am_i_reg = LSM6DSOX_WHO_AM_I;
    uint8_t who_am_i_value = 0;
    int ret;

    ret = i2c_write_read_dt(&dev_i2c, &who_am_i_reg, 1, &who_am_i_value, 1);
    if (ret) {
        LOG_ERR("whoami read failed: %d", ret);
        return;
    }

    if (who_am_i_value != 0x6C) {
        LOG_ERR("whoami mismatch: expected 0x6C, got 0x%02X", who_am_i_value);
    } else {
        LOG_INF("whoami: 0x%02X", who_am_i_value);
    }
}

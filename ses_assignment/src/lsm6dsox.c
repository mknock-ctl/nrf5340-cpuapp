#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "lsm6dsox.h"

LOG_MODULE_REGISTER(lsm6dsox, LOG_LEVEL_DBG);

#define LSM6DSOX_NODE DT_NODELABEL(lsm6dsox)

static const struct i2c_dt_spec dev = I2C_DT_SPEC_GET(LSM6DSOX_NODE);

int lsm6dsox_write_reg(uint8_t reg, uint8_t value)
{
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    uint8_t buf[2] = {reg, value};
    return i2c_write_dt(&dev, buf, sizeof(buf));
}

int lsm6dsox_read_reg(uint8_t reg, uint8_t *value)
{
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    return i2c_write_read_dt(&dev, &reg, 1, value, 1);
}

int lsm6dsox_verify_device(void)
{
    uint8_t who_am_i;
    int ret = lsm6dsox_read_reg(LSM6DSOX_WHO_AM_I, &who_am_i); if (ret) { return ret; }

    if (who_am_i != LSM6DSOX_WHO_AM_I_VALUE) {
        LOG_ERR("dev id not match: 0x%02X", who_am_i);
        return -ENODEV;
    }

    LOG_INF("dev verified: 0x%02X", who_am_i);
    return 0;
}

static int lsm6dsox_configure_tap(void)
{
    struct {
        uint8_t reg;
        uint8_t val;
    } config[] = {
        {LSM6DSOX_CTRL1_XL,    0x60},
        {LSM6DSOX_CTRL3_C,     0x44},
        {LSM6DSOX_CTRL4_C,     0x00},
        {LSM6DSOX_TAP_CFG0,    0x8E},
        {LSM6DSOX_TAP_CFG1,    0x00},
        {LSM6DSOX_TAP_CFG2,    0x80},
        {LSM6DSOX_TAP_THS_6D,  0x8C},
        {LSM6DSOX_INT_DUR2,    0x7F},
        {LSM6DSOX_WAKE_UP_THS, 0x00},
        {LSM6DSOX_MD1_CFG,     0x48},
    };

    for (size_t i = 0; i < ARRAY_SIZE(config); i++) {
        int ret = lsm6dsox_write_reg(config[i].reg, config[i].val);
        if (ret) {
            return ret;
        }
    }

    return 0;
}

int lsm6dsox_init(void)
{
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    int ret = lsm6dsox_verify_device(); if (ret) { return ret; }
    ret = lsm6dsox_configure_tap(); if (ret) { return ret; }

    LOG_INF("tap detection configured");
    return 0;
}
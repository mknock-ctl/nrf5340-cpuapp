#include "robot/sensors/lis3mdl.h"
#include "ses_assignment.h"
#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(lis3mdl, LOG_LEVEL_DBG);

static const struct i2c_dt_spec dev = I2C_DT_SPEC_GET(LIS3MDL_NODE);


int lis3mdl_write_reg(uint8_t reg, uint8_t value) {
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    uint8_t buf[2] = {reg, value};
    return i2c_write_dt(&dev, buf, sizeof(buf));
}

int lis3mdl_read_multi_reg(uint8_t reg, uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return -EINVAL;
    }

    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }
    // Set MSB for auto increment read (0x80)
    uint8_t reg_addr = reg | BIT(7);
    return i2c_write_read_dt(&dev, &reg_addr, 1, data, len);
}

int lis3mdl_verify_device(void) {
    uint8_t who_am_i;
    TRY_ERR(int, i2c_write_read_dt(&dev, (uint8_t[]){LIS3MDL_WHO_AM_I}, 1, &who_am_i, 1));

    if (who_am_i != LIS3MDL_WHO_AM_I_VALUE) {
        LOG_ERR("LIS3MDL dev id not match: 0x%02X", who_am_i);
        return -ENODEV;
    }

    LOG_INF("LIS3MDL dev verified: 0x%02X", who_am_i);
    return 0;
}

int lis3mdl_init(void) {
    if (!device_is_ready(dev.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    TRY_ERR(int, lis3mdl_verify_device());
    
    CONFIGURE_REGS(lis3mdl_write_reg, {LIS3MDL_CTRL_REG1, LIS3MDL_OM_HIGH_PERF | LIS3MDL_ODR_80HZ},
                   {LIS3MDL_CTRL_REG2, LIS3MDL_FS_4_GAUSS},
                   {LIS3MDL_CTRL_REG3, LIS3MDL_MD_CONTINUOUS},
                   {LIS3MDL_CTRL_REG4, LIS3MDL_OMZ_HIGH_PERF | LIS3MDL_BLE_LSB},
                   {LIS3MDL_CTRL_REG5, LIS3MDL_BDU_CONTINUOUS})

    LOG_INF("LIS3MDL configured for continuous mode");
    return 0;
}

int lis3mdl_read_mag(lis3mdl_data_t *data) {
    uint8_t status;
    TRY_ERR(int, lis3mdl_read_multi_reg(LIS3MDL_STATUS_REG, &status, 1));
    
    uint8_t buf[6];
    TRY_ERR(int, lis3mdl_read_multi_reg(LIS3MDL_OUT_X_L, buf, sizeof(buf)));
    
    data->x = (int16_t)(buf[0] | (buf[1] << 8));
    data->y = (int16_t)(buf[2] | (buf[3] << 8));
    data->z = (int16_t)(buf[4] | (buf[5] << 8));

    //LOG_DBG("Mag X:%d, Y:%d, Z:%d", data->x, data->y, data->z);
    //LOG_DBG("Mag X:%.3f, Y:%.3f", (double) (data->x * (1.0 / 6842.0)) , (double) (data->y * (1.0 / 6842.0)));
    return 0;
}


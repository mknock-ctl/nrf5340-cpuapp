#include "lsm6dsox.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include "ses_assignment.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(lsm6dsox, LOG_LEVEL_DBG);

#define LSM6DSOX_NODE DT_NODELABEL(lsm6dsox)

static const struct i2c_dt_spec dev = I2C_DT_SPEC_GET(LSM6DSOX_NODE);

int lsm6dsox_write_reg(uint8_t reg, uint8_t value) {
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    uint8_t buf[2] = {reg, value};
    return i2c_write_dt(&dev, buf, sizeof(buf));
}

int lsm6dsox_read_reg(uint8_t reg, uint8_t *value) {
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    return i2c_write_read_dt(&dev, &reg, 1, value, 1);
}

int lsm6dsox_verify_device(void) {
    uint8_t who_am_i;
    int ret = lsm6dsox_read_reg(LSM6DSOX_WHO_AM_I, &who_am_i);
    if (ret) {
        return ret;
    }

    if (who_am_i != LSM6DSOX_WHO_AM_I_VALUE) {
        LOG_ERR("dev id not match: 0x%02X", who_am_i);
        return -ENODEV;
    }

    LOG_INF("dev verified: 0x%02X", who_am_i);
    return 0;
}

static int lsm6dsox_configure_tap(void) {
    struct {
        uint8_t reg;
        uint8_t val;
    } config[] = {
        // CONTROL
        // 0x60 = 0b0110_0000
        // ODR_XL = 0110 (416 Hz), FS_XL = 00 (±2g), LPF2_EN=0
        {LSM6DSOX_CTRL1_XL, 0x60},

        // 0x44 = 0b0100_0100
        // BDU = 1 (Block Data Update)
        // IF_INC = 1 (auto-increment addresses)
        {LSM6DSOX_CTRL3_C, 0x44},

        // 0x00 = 0b0000_0000
        // All default: I2C enable, no mirror, no special mode
        {LSM6DSOX_CTRL4_C, 0x00},

        // 0x4F = 0b0100_1111
        // bit7 = 0 (reserved)
        // bit6 = 1 → INT_CLR_ON_READ
        // bit5 = 0
        // bit4 = 0
        // bit3 = 1 → TAP_X_EN
        // bit2 = 1 → TAP_Y_EN
        // bit1 = 1 → TAP_Z_EN
        // bit0 = 1 → LIR (latched interrupt)
        {LSM6DSOX_TAP_CFG0, 0x4F},

        // 0x0C = 0b0000_1100
        // bits[7:5] = 000 (TAP_PRIORITY default)
        // bits[4:0] = 01100 → TAP_THS_X = 12 (≈0.96 g for ±2g FS)
        {LSM6DSOX_TAP_CFG1, 0x0C},

        // 0x80 = 0b1000_0000
        // bit7 = 1 → INTERRUPTS_ENABLE
        // bits[6:5] = 00 (inactivity disabled)
        // bits[4:0] = 00000 → TAP_THS_Y = 0 (use Z only or adjust)
        {LSM6DSOX_TAP_CFG2, 0x80},

        // 0x8C = 0b1000_1100
        // bit7 = 1 → 4D/6D enabled
        // bits[6:5] = 00 (SIXD_THS default)
        // bits[4:0] = 01100 → TAP_THS_Z = 12 (≈0.96 g)
        {LSM6DSOX_TAP_THS_6D, 0x8C},

        // 0x7F = 0b0111_1111
        // bits[7:4] = 0111 → DUR = 7
        // bits[3:2] = 11 → QUIET = 3
        // bits[1:0] = 11 → SHOCK = 3
        {LSM6DSOX_INT_DUR2, 0x7F},

        // 0x00 = 0b0000_0000
        // Wakeup detection disabled
        {LSM6DSOX_WAKE_UP_THS, 0x00},

        // 0x48 = 0b0100_1000
        // bit6 = 1 → INT1_SINGLE_TAP
        // bit3 = 1 → INT1_DOUBLE_TAP
        // Others = 0
        {LSM6DSOX_MD1_CFG, 0x48},
    };

    for (size_t i = 0; i < ARRAY_SIZE(config); i++) {
        int ret = lsm6dsox_write_reg(config[i].reg, config[i].val);
        if (ret) {
            return ret;
        }
    }

    return 0;
}


int lsm6dsox_read_gyro(lsm6dsox_gyro_data_t *data) {
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }
    
    uint8_t buf[6];
    uint8_t reg = LSM6DSOX_OUTX_L_G;
    
    TRY_ERR(int, i2c_write_read_dt(&dev, &reg, 1, buf, 6));
    
    data->x = (int16_t)(buf[1] << 8 | buf[0]);
    data->y = (int16_t)(buf[3] << 8 | buf[2]);
    data->z = (int16_t)(buf[5] << 8 | buf[4]);
    
    return 0;
}

float lsm6dsox_gyro_to_dps(int16_t raw_value) {
    // For ±500 dps range: sensitivity = 17.50 mdps/LSB
    // For ±250 dps range: sensitivity = 8.75 mdps/LSB
    // For ±1000 dps range: sensitivity = 35.0 mdps/LSB
    const float sensitivity = 17.50f;  // mdps/LSB for ±500 dps
    return (float)raw_value * sensitivity / 1000.0f;  // Convert to dps
}


int lsm6dsox_init(void) {
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    int ret = lsm6dsox_verify_device();
    if (ret) {
        return ret;
    }
    ret = lsm6dsox_configure_tap();
    if (ret) {
        return ret;
    }

    LOG_INF("tap detection configured");
    return 0;
}

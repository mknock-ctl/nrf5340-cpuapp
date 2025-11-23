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
    TRY_ERR(int, lsm6dsox_read_reg(LSM6DSOX_WHO_AM_I, &who_am_i));

    if (who_am_i != LSM6DSOX_WHO_AM_I_VALUE) {
        LOG_ERR("dev id not match: 0x%02X", who_am_i);
        return -ENODEV;
    }

    LOG_INF("dev verified: 0x%02X", who_am_i);
    return 0;
}

static int lsm6dsox_configure_tap(void) {
    CONFIGURE_REGS(lsm6dsox_write_reg,
        // 1. Setup ODR and Range
        {LSM6DSOX_CTRL1_XL, ODR_XL_416Hz | FS_XL_2g},

        // 2. Enable Block Data Update
        {LSM6DSOX_CTRL3_C, 0x44}, // BDU | IF_INC

        // 3. Tap Configuration (Z-axis enabled, Latch Interrupt Request ON)
        // TAP_LIR is critical: It keeps INT1 high until we read TAP_SRC
        {LSM6DSOX_TAP_CFG0, TAP_Z_EN | TAP_X_EN | TAP_Y_EN | TAP_LIR},

        // 4. X Threshold (Increased slightly to 0x09 to reduce noise triggers)
        {LSM6DSOX_TAP_CFG1, TAP_THRESHOLD_X(0x08)}, 

        // 5. Enable Interrupts in hardware engine
        {LSM6DSOX_TAP_CFG2, TAP_INTERRUPTS_ENABLE},

        // 6. Z Threshold (0x09 is approx 500mg on 2g scale - standard for finger taps)
        {LSM6DSOX_TAP_THS_6D, TAP_THRESHOLD_Z(0x08)}, 

        // Duration: 0x06 (Wait ~300ms for second tap)
        // Quiet:    0x02 (Wait ~20ms after tap before listening again. Short is better)
        // Shock:    0x02 (Wait ~40ms max for peak impact)
        {LSM6DSOX_INT_DUR2, TAP_DURATION(0x07) | TAP_QUIET(0x01) | TAP_SHOCK(0x02)},

        // 8. Route Double Tap to INT1
        // Note: We ONLY route Double Tap. Single tap will update the register
        // but won't fire the physical pin. This reduces ISR overhead.
        {LSM6DSOX_MD1_CFG, INT1_DOUBLE_TAP})

    return 0;
}

int lsm6dsox_configure_gyro(void) {
    CONFIGURE_REGS(lsm6dsox_write_reg, 
        {LSM6DSOX_CTRL2_G, GYRO_CFG_500DPS_416HZ})

    LOG_INF("Gyroscope configured");
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

void lsm6dsox_clear_interrupts(void) {
    uint8_t dummy;
    lsm6dsox_read_reg(LSM6DSOX_ALL_INT_SRC, &dummy);
    lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &dummy);
    LOG_DBG("Interrupts cleared on boot");
}


int lsm6dsox_init(void) {
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    TRY_ERR(int, lsm6dsox_verify_device());
    TRY_ERR(int, lsm6dsox_configure_tap());
    TRY_ERR(int, lsm6dsox_configure_gyro());

    lsm6dsox_clear_interrupts();

    LOG_INF("tap detection configured");
    return 0;
}

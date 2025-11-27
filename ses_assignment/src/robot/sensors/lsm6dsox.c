#include "robot/sensors/lsm6dsox.h"
#include "ses_assignment.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include "robot/sensors/int1_gpio.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
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

int lsm6dsox_read_multi_reg(uint8_t reg, uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return -EINVAL;
    }

    const struct i2c_dt_spec dev = I2C_DT_SPEC_GET(DT_NODELABEL(lsm6dsox));
    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    return i2c_write_read_dt(&dev, &reg, 1, data, len);
}

int lsm6dsox_update_reg(uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t old_val;
    TRY_ERR(int, lsm6dsox_read_reg(reg, &old_val));
    uint8_t new_val = (old_val & ~mask) | (val & mask);
    return lsm6dsox_write_reg(reg, new_val);
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

int lsm6dsox_read_accel_raw(int16_t *x, int16_t *y, int16_t *z) {
    if (!x || !y || !z) {
        return -EINVAL;
    }

    if (!device_is_ready(dev.bus)) {
        return -ENODEV;
    }

    uint8_t buf[6];
    uint8_t reg = LSM6DSOX_OUTX_L_A;

    TRY_ERR(int, i2c_write_read_dt(&dev, &reg, 1, buf, 6));

    *x = (int16_t)(buf[1] << 8 | buf[0]);
    *y = (int16_t)(buf[3] << 8 | buf[2]);
    *z = (int16_t)(buf[5] << 8 | buf[4]);

    return 0;
}

int lsm6dsox_route_dataready_int1(bool enable) {
    return lsm6dsox_route_int1(INT1_DRDY_XL, enable);
}

int lsm6dsox_route_int1(uint8_t bit_mask, bool enable) {
    // This allows multiple interrupts to be OR'd together on INT1
    // or cleanly removed without resetting the whole register.
    return lsm6dsox_update_reg(LSM6DSOX_MD1_CFG, 
                               bit_mask, 
                               enable ? bit_mask : 0);
}

int lsm6dsox_reset_embedded_functions(void) {
    // Disable the master interrupt engine first to stop any firing
    lsm6dsox_write_reg(LSM6DSOX_TAP_CFG2, 0x00);
    
    // Clear all other related configuration registers
    lsm6dsox_write_reg(LSM6DSOX_TAP_CFG0, 0x00);
    lsm6dsox_write_reg(LSM6DSOX_TAP_CFG1, 0x00);
    lsm6dsox_write_reg(LSM6DSOX_TAP_THS_6D, 0x00);
    lsm6dsox_write_reg(LSM6DSOX_INT_DUR2, 0x00);
    lsm6dsox_write_reg(LSM6DSOX_WAKE_UP_THS, 0x00);
    lsm6dsox_write_reg(LSM6DSOX_WAKE_UP_DUR, 0x00);
    lsm6dsox_write_reg(LSM6DSOX_MD1_CFG, 0x00); // Disconnect INT1 routing

    // Clear any pending interrupts
    lsm6dsox_clear_interrupts();
    return 0;
}

int lsm6dsox_configure_tap_params(void) {
    LOG_INF("Configuring tap detection...");
    lsm6dsox_reset_embedded_functions();

    CONFIGURE_REGS(lsm6dsox_write_reg,
                   // Setup ODR and Range
                   {LSM6DSOX_CTRL1_XL, ODR_XL_416Hz | FS_XL_2g},

                   // Enable Block Data Update
                   {LSM6DSOX_CTRL3_C, 0x44}, // BDU | IF_INC

                   // Enable INT1 output (some LSM6DSOX require this)
                   {LSM6DSOX_CTRL4_C, 0x00},

                   // Tap Configuration - ONLY Z-axis for cleaner detection
                   // TAP_LIR keeps INT1 high until we read TAP_SRC
                   {LSM6DSOX_TAP_CFG0, TAP_Z_EN | TAP_LIR},

                   // X Threshold (not used since X is disabled, but set anyway)
                   {LSM6DSOX_TAP_CFG1, TAP_THRESHOLD_X(0x08)},

                   // Enable Interrupts in hardware engine
                   {LSM6DSOX_TAP_CFG2, TAP_INTERRUPTS_ENABLE},

                   // Z Threshold - Balanced sensitivity
                   {LSM6DSOX_TAP_THS_6D, TAP_THRESHOLD_Z(0x0A)},

                   // Single/double-tap selection and wake-up configuration (R/W)
                   {LSM6DSOX_WAKE_UP_THS, WAKE_UP_THS_SINGLE_DOUBLE_TAP | CRASH_THRESHOLD_WAKEUP},

                   // SHOCK: 0x01 = ~20ms (tap must END quickly so sensor knows it's done)
                   // QUIET: 0x01 = ~20ms (brief pause after tap before listening again)
                   // DUR:   0x06 = ~300ms (maximum time between the two taps)
                   //
                   // Problem with your settings: SHOCK=0x03 is TOO LONG
                   // The sensor thinks the tap never ends, so it can't detect a "second" tap
                   {LSM6DSOX_INT_DUR2, TAP_DURATION(0x06) | TAP_QUIET(0x02) | TAP_SHOCK(0x02)},

                   {LSM6DSOX_WAKE_UP_DUR, 0x00});

    LOG_INF("Tap detection configured successfully");

    return 0;
}

int lsm6dsox_configure_crash_params(void) {
    LOG_INF("Configuring crash params...");

    lsm6dsox_reset_embedded_functions();

    CONFIGURE_REGS(lsm6dsox_write_reg,
                   // Basic setup
                   {LSM6DSOX_CTRL1_XL, ODR_XL_416Hz | FS_XL_2g},
                   {LSM6DSOX_CTRL3_C, 0x44}, 
                   
                   {LSM6DSOX_TAP_CFG0, 0x00}, 

                   {LSM6DSOX_WAKE_UP_THS, CRASH_THRESHOLD_WAKEUP}, 

                   {LSM6DSOX_WAKE_UP_DUR, 0x02},

                   {LSM6DSOX_TAP_CFG2, TAP_INTERRUPTS_ENABLE});

    return 0;
}

int lsm6dsox_configure_gyro(void) {
    CONFIGURE_REGS(lsm6dsox_write_reg, {LSM6DSOX_CTRL2_G, GYRO_CFG_500DPS_416HZ})

    LOG_INF("Gyroscope configured");
    return 0;
}

int lsm6dsox_accel_set_odr(bool active) {
    if (active) {
        // Set ODR to 416Hz, 2g scale (High Performance)
        return lsm6dsox_update_reg(LSM6DSOX_CTRL1_XL, 
                                 0xF0, // Mask for ODR
                                 ODR_XL_416Hz | FS_XL_2g);
    } else {
        // Power down accelerometer (ODR = 0)
        return lsm6dsox_update_reg(LSM6DSOX_CTRL1_XL, 0xF0, 0x00);
    }
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
    const float sensitivity = 17.50f;                // mdps/LSB for ±500 dps
    return (float)raw_value * sensitivity / 1000.0f; // Convert to dps
}

void lsm6dsox_clear_interrupts(void) {
    uint8_t dummy;
    lsm6dsox_read_reg(LSM6DSOX_ALL_INT_SRC, &dummy);
    lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &dummy);
    lsm6dsox_read_reg(LSM6DSOX_WAKE_UP_SRC, &dummy);
}

int lsm6dsox_init(void) {
    if (!device_is_ready(dev.bus)) return -ENODEV;
    LOG_INF("Initializing LSM6DSOX...");
    TRY_ERR(int, lsm6dsox_verify_device());

    // IMPORTANT: Reset the device to clear any stuck interrupts from previous boot
    lsm6dsox_update_reg(LSM6DSOX_CTRL3_C, 0x01, 0x01);
    k_busy_wait(10000);

    // Reset BDU/IF_INC after reset
    lsm6dsox_write_reg(LSM6DSOX_CTRL3_C, 0x44);

    TRY_ERR(int, lsm6dsox_configure_gyro()); // Gyro is always on
    lsm6dsox_clear_interrupts();
    return 0;
}

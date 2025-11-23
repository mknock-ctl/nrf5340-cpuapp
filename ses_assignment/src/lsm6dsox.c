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
    LOG_INF("Configuring tap detection...");
    
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
        {LSM6DSOX_TAP_THS_6D, TAP_THRESHOLD_Z(0x08)}, 

        // Single/double-tap selection and wake-up configuration (R/W)
        {LSM6DSOX_WAKE_UP_THS, WAKE_UP_THS_SINGLE_DOUBLE_TAP},
        
        // 8. CRITICAL TIMING for double-tap recognition:
        // SHOCK: 0x01 = ~20ms (tap must END quickly so sensor knows it's done)
        // QUIET: 0x01 = ~20ms (brief pause after tap before listening again)
        // DUR:   0x06 = ~300ms (maximum time between the two taps)
        //
        // Problem with your settings: SHOCK=0x03 is TOO LONG
        // The sensor thinks the tap never ends, so it can't detect a "second" tap
        {LSM6DSOX_INT_DUR2, TAP_DURATION(0x08) | TAP_QUIET(0x01) | TAP_SHOCK(0x02)},
        
        // 9. Route ONLY Double Tap to INT1
        {LSM6DSOX_MD1_CFG, INT1_DOUBLE_TAP})
    
    LOG_INF("Tap detection configured successfully");
    LOG_INF("  SHOCK=0x01 (~20ms), QUIET=0x01 (~20ms), DUR=0x06 (~300ms)");
    
    return 0;
}

void lsm6dsox_debug_tap_config(void) {
    uint8_t reg_val;
    
    LOG_INF("=== LSM6DSOX Tap Configuration Debug ===");
    
    // Read back all tap-related registers
    lsm6dsox_read_reg(LSM6DSOX_CTRL1_XL, &reg_val);
    LOG_INF("CTRL1_XL (0x10):    0x%02X", reg_val);
    
    lsm6dsox_read_reg(LSM6DSOX_CTRL3_C, &reg_val);
    LOG_INF("CTRL3_C (0x12):     0x%02X", reg_val);
    
    lsm6dsox_read_reg(LSM6DSOX_TAP_CFG0, &reg_val);
    LOG_INF("TAP_CFG0 (0x56):    0x%02X (Z_EN=%d, Y_EN=%d, X_EN=%d, LIR=%d)", 
            reg_val, 
            !!(reg_val & (1<<1)),
            !!(reg_val & (1<<2)),
            !!(reg_val & (1<<3)),
            !!(reg_val & (1<<0)));
    
    lsm6dsox_read_reg(LSM6DSOX_TAP_CFG1, &reg_val);
    LOG_INF("TAP_CFG1 (0x57):    0x%02X (X_THS=0x%02X)", reg_val, reg_val & 0x1F);
    
    lsm6dsox_read_reg(LSM6DSOX_TAP_CFG2, &reg_val);
    LOG_INF("TAP_CFG2 (0x58):    0x%02X (INTERRUPTS_EN=%d)", reg_val, !!(reg_val & 0x80));
    
    lsm6dsox_read_reg(LSM6DSOX_TAP_THS_6D, &reg_val);
    LOG_INF("TAP_THS_6D (0x59):  0x%02X (Z_THS=0x%02X)", reg_val, reg_val & 0x1F);
    
    lsm6dsox_read_reg(LSM6DSOX_INT_DUR2, &reg_val);
    LOG_INF("INT_DUR2 (0x5A):    0x%02X (DUR=%d, QUIET=%d, SHOCK=%d)", 
            reg_val,
            (reg_val >> 4) & 0x0F,
            (reg_val >> 2) & 0x03,
            reg_val & 0x03);
    
    lsm6dsox_read_reg(LSM6DSOX_MD1_CFG, &reg_val);
    LOG_INF("MD1_CFG (0x5E):     0x%02X (DOUBLE_TAP=%d, SINGLE_TAP=%d)", 
            reg_val,
            !!(reg_val & (1<<3)),
            !!(reg_val & (1<<6)));
    
    // Check current interrupt status
    lsm6dsox_read_reg(LSM6DSOX_TAP_SRC, &reg_val);
    LOG_INF("TAP_SRC (0x1C):     0x%02X", reg_val);
    
    lsm6dsox_read_reg(LSM6DSOX_ALL_INT_SRC, &reg_val);
    LOG_INF("ALL_INT_SRC (0x1D): 0x%02X", reg_val);
    
    LOG_INF("========================================");
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

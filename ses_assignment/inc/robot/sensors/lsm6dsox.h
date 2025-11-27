#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdint.h>
#include <stdbool.h>

#define LSM6DSOX_ADDR 0x6A
#define LSM6DSOX_WHO_AM_I 0x0F
#define LSM6DSOX_WHO_AM_I_VALUE 0x6C
#define LSM6DSOX_INT1_CTRL 0x0D
#define LSM6DSOX_OUTX_L_A 0x28
#define LSM6DSOX_CTRL1_XL 0x10
#define LSM6DSOX_CTRL2_G 0x11
#define LSM6DSOX_CTRL3_C 0x12
#define LSM6DSOX_CTRL4_C 0x13
#define LSM6DSOX_TAP_CFG0 0x56
#define LSM6DSOX_TAP_CFG1 0x57
#define LSM6DSOX_TAP_CFG2 0x58
#define LSM6DSOX_TAP_THS_6D 0x59
#define LSM6DSOX_INT_DUR2 0x5A
#define LSM6DSOX_WAKE_UP_THS 0x5B
#define LSM6DSOX_WAKE_UP_DUR 0x5C
#define LSM6DSOX_MD1_CFG 0x5E
#define LSM6DSOX_WAKE_UP_SRC 0x1B
#define LSM6DSOX_TAP_SRC 0x1C
#define LSM6DSOX_ALL_INT_SRC 0x1D
#define LSM6DSOX_OUTX_L_G 0x22

// CTRL1_XL
#define ODR_XL_416Hz (0x06 << 4)
#define FS_XL_2g (0x00 << 2)

// TAP_CFG0
#define TAP_LIR (1 << 0)
#define TAP_X_EN (1 << 3)
#define TAP_Y_EN (1 << 2)
#define TAP_Z_EN (1 << 1)

// TAP_CFG1 & TAP_THS_6D
#define TAP_THRESHOLD_X(val) (val & 0x1F)
#define TAP_THRESHOLD_Z(val) (val & 0x1F)

// TAP_CFG2
#define TAP_INTERRUPTS_ENABLE (1 << 7)

#define INT1_DRDY_XL (1 << 0)
// INT_DUR2
// Dur: Max time between two taps
// Quiet: Time to ignore inputs after a tap (Keep this short!)
// Shock: Max duration of the impact
#define TAP_DURATION(val) ((val & 0x0F) << 4)
#define TAP_QUIET(val) ((val & 0x03) << 2)
#define TAP_SHOCK(val) (val & 0x03)

// MD1_CFG
#define INT1_DOUBLE_TAP (1 << 3)
#define INT1_WU         (1 << 5)
#define INT1_SINGLE_TAP (1 << 6)
#define INT1_6D         (1 << 2)

// TAP SRC Flags
#define TAP_SRC_DOUBLE_TAP (1 << 4)
#define TAP_SRC_SINGLE_TAP (1 << 5)

// WAKE_UP_THS
#define WAKE_UP_THS_SINGLE_DOUBLE_TAP (1 << 7)
#define CRASH_THRESHOLD_WAKEUP 0x19 // On ground 0x1C
// #define CRASH_THRESHOLD_WAKEUP 0x19 // On table
// gyro values

#define GYRO_CFG_500DPS_416HZ 0x64  // 0b0110_0100
#define GYRO_CFG_250DPS_416HZ 0x60  // 0b0110_0000 (if you want more precision)
#define GYRO_CFG_1000DPS_416HZ 0x68 // 0b0110_1000 (if turns are very fast)

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lsm6dsox_gyro_data_t;

int lsm6dsox_init(void);
int lsm6dsox_verify_device(void);

int lsm6dsox_read_reg(uint8_t reg, uint8_t *value);
int lsm6dsox_write_reg(uint8_t reg, uint8_t value);
int lsm6dsox_update_reg(uint8_t reg, uint8_t mask, uint8_t value);

int lsm6dsox_route_int1(uint8_t bit_mask, bool enable);
int lsm6dsox_route_dataready_int1(bool enable);
int lsm6dsox_read_accel_raw(int16_t *x, int16_t *y, int16_t *z);

int lsm6dsox_configure_tap_params(void);
int lsm6dsox_configure_crash_params(void);
int lsm6dsox_configure_gyro(void);

float lsm6dsox_gyro_to_dps(int16_t raw_value);
int lsm6dsox_read_gyro(lsm6dsox_gyro_data_t *data);
void lsm6dsox_clear_interrupts(void);

#endif

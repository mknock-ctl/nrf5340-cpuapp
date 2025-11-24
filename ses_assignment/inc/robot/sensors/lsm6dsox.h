#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdint.h>

#define LSM6DSOX_ADDR 0x6A
#define LSM6DSOX_WHO_AM_I 0x0F
#define LSM6DSOX_WHO_AM_I_VALUE 0x6C

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
#define LSM6DSOX_MD1_CFG 0x5E
#define LSM6DSOX_ALL_INT_SRC 0x1D
#define LSM6DSOX_TAP_SRC 0x1C
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

// INT_DUR2
// Dur: Max time between two taps
// Quiet: Time to ignore inputs after a tap (Keep this short!)
// Shock: Max duration of the impact
#define TAP_DURATION(val) ((val & 0x0F) << 4)
#define TAP_QUIET(val) ((val & 0x03) << 2)
#define TAP_SHOCK(val) (val & 0x03)

// MD1_CFG
#define INT1_DOUBLE_TAP (1 << 3)
#define INT1_SINGLE_TAP (1 << 6)

// TAP SRC Flags
#define TAP_SRC_DOUBLE_TAP (1 << 4)
#define TAP_SRC_SINGLE_TAP (1 << 5)

// WAKE_UP_THS
#define WAKE_UP_THS_SINGLE_DOUBLE_TAP (1 << 7)

// gyro values

#define GYRO_CFG_500DPS_416HZ 0x64  // 0b0110_0100
#define GYRO_CFG_250DPS_416HZ 0x60  // 0b0110_0000 (if you want more precision)
#define GYRO_CFG_1000DPS_416HZ 0x68 // 0b0110_1000 (if turns are very fast)

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lsm6dsox_gyro_data_t;
void lsm6dsox_clear_interrupts(void);

int lsm6dsox_init(void);
int lsm6dsox_verify_device(void);
int lsm6dsox_read_reg(uint8_t reg, uint8_t *value);
float lsm6dsox_gyro_to_dps(int16_t raw_value);
int lsm6dsox_read_gyro(lsm6dsox_gyro_data_t *data);

#endif

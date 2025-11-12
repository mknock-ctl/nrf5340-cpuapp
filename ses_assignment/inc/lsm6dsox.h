#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdint.h>

#define LSM6DSOX_ADDR           0x6A
#define LSM6DSOX_WHO_AM_I       0x0F
#define LSM6DSOX_WHO_AM_I_VALUE 0x6C

#define LSM6DSOX_CTRL1_XL       0x10
#define LSM6DSOX_CTRL3_C        0x12
#define LSM6DSOX_CTRL4_C        0x13
#define LSM6DSOX_TAP_CFG0       0x56
#define LSM6DSOX_TAP_CFG1       0x57
#define LSM6DSOX_TAP_CFG2       0x58
#define LSM6DSOX_TAP_THS_6D     0x59
#define LSM6DSOX_INT_DUR2       0x5A
#define LSM6DSOX_WAKE_UP_THS    0x5B
#define LSM6DSOX_WAKE_UP_DUR    0x5C
#define LSM6DSOX_MD1_CFG        0x5E
#define LSM6DSOX_ALL_INT_SRC    0x1D
#define LSM6DSOX_TAP_SRC        0x1C

#define TAP_SRC_DOUBLE_TAP      (1 << 5)
#define TAP_SRC_SINGLE_TAP      (1 << 4)

int lsm6dsox_init(void);
int lsm6dsox_verify_device(void);
int lsm6dsox_write_reg(uint8_t reg, uint8_t value);
int lsm6dsox_read_reg(uint8_t reg, uint8_t *value);

#endif
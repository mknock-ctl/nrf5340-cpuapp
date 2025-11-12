#ifndef I2C
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

int lsm6dsox_write_reg(uint8_t reg, uint8_t value);
int lsm6dsox_read_reg(uint8_t reg, uint8_t *value);
int check_i2c_bus();

#endif
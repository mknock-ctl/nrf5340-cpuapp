
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(i2c_helper, LOG_LEVEL_DBG);

#define I2C1_NODE DT_NODELABEL(lsm6dsox)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

int lsm6dsox_write_reg(uint8_t reg, uint8_t value) // over i2c
{
    uint8_t buf[2] = { reg, value };
    return i2c_write_dt(&dev_i2c, buf, sizeof(buf));
}

int lsm6dsox_read_reg(uint8_t reg, uint8_t *value) // over i2c
{
    return i2c_write_read_dt(&dev_i2c, &reg, 1, value, 1);
}

int check_i2c_bus(){
    if (!device_is_ready(dev_i2c.bus)){
        LOG_ERR("I2C bus %s not ready", dev_i2c.bus ? dev_i2c.bus->name : "unknown");
        return -ENODEV;
    }
    return 0;
}
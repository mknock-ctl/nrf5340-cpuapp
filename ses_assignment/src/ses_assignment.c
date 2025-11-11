/**
 * @file ses_assignment.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Stubs for SES assignment
 * @date 2025-10-01
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "ses_assignment.h"
#include <mergebot.h>

LOG_MODULE_REGISTER(ses_assignment, LOG_LEVEL_DBG);

#define I2C1_NODE DT_NODELABEL(lsm6dsox)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

void move(int32_t distance) { 
    //LOG_ERR("move is not yet implemented"); 
}

void turn(int32_t angle) { 
    //LOG_ERR("turn is not yet implemented"); 
}

void turn_to_north(void) { 
    //LOG_ERR("turn_to_north is not yet implemented");
}

void wait_for_double_tap(void) { 
    //LOG_ERR("wait_for_double_tap is not yet implemented");
}

void update_status(ses_status_code code) {
    //LOG_ERR("set_status_led is not yet implemented");
}

void imu_init(void)
{
    if (!device_is_ready(dev_i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
        return;
    }
}

void imu_test(void)
{
    uint8_t who_am_i_reg = 0x0F;
    uint8_t who_am_i_value = 0;
    int ret = 0;

    ret = i2c_write_read_dt(&dev_i2c, &who_am_i_reg, sizeof(who_am_i_reg), 
                        &who_am_i_value, sizeof(who_am_i_value));
    
    if (ret != 0) {
        LOG_ERR("failed to get whoami register, e: %d", ret);
        return;
    }

    if (who_am_i_value != 0x6C) {
        LOG_ERR("whoami register has wrong value. expected: 0x6C, Got: 0x%02X", who_am_i_value);
    } else {
        LOG_INF("whoami register read success: 0x%02X", who_am_i_value);
    }
}

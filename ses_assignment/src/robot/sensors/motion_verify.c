#include "robot/sensors/motion_verify.h"
#include "robot/sensors/lsm6dsox.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ses_assignment.h"
#include <math.h>

LOG_MODULE_REGISTER(motion_verify, LOG_LEVEL_DBG);

static motion_status_handler_t status_callback = NULL;
static volatile bool is_active = false;
static int16_t expected_speed = 0;
static struct k_timer verify_timer;
static struct k_work verify_work;
static motion_status_t last_status = MOTION_OK;

#define VELOCITY_THRESHOLD 50.0f
#define ACCEL_SCALE_2G 0.061f

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accel_data_t;

static int read_accelerometer(accel_data_t *data) {
    uint8_t buf[6];
    
    TRY_ERR(int, lsm6dsox_read_multi_reg(LSM6DSOX_OUTX_L_A, buf, sizeof(buf)));

    data->x = (int16_t)(buf[1] << 8 | buf[0]);
    data->y = (int16_t)(buf[3] << 8 | buf[2]);
    data->z = (int16_t)(buf[5] << 8 | buf[4]);

    return 0;
}


static void verify_work_handler(struct k_work *work) {
    if (!is_active || expected_speed == 0) {
        return;
    }

    accel_data_t accel;
    TRY_ERR(int, read_accelerometer(&accel));

    float accel_x_ms2 = (float)accel.x * ACCEL_SCALE_2G * 9.81f / 1000.0f;
    float expected_accel = 0.0f;
    
    float accel_magnitude = fabsf(accel_x_ms2);
    
    motion_status_t new_status = MOTION_OK;
    
    if (expected_speed > 0) {
        if (accel_x_ms2 > 0.5f) {
            // (downhill/slip)
            new_status = MOTION_FAST;
        } else if (accel_x_ms2 < -0.5f) {
            // (uphill/drag)
            new_status = MOTION_SLOW;
        }
    } else if (expected_speed < 0) {
        if (accel_x_ms2 < -0.5f) {
            new_status = MOTION_FAST;
        } else if (accel_x_ms2 > 0.5f) {
            new_status = MOTION_SLOW;
        }
    }

    // Only trigger callback if status changed
    if (new_status != last_status) {
        last_status = new_status;
        if (status_callback) {
            LOG_INF("Motion status changed: %d (accel_x: %.2f m/s²)", 
                    new_status, (double)accel_x_ms2);
            status_callback(new_status);
        }
    }
}

static void verify_timer_handler(struct k_timer *timer) {
    k_work_submit(&verify_work);
}

int motion_verify_init(motion_status_handler_t handler) {
    if (!handler) return -EINVAL;

    status_callback = handler;
    is_active = false;
    last_status = MOTION_OK;
    
    k_work_init(&verify_work, verify_work_handler);
    k_timer_init(&verify_timer, verify_timer_handler, NULL);
    
    LOG_INF("Motion verification initialized");
    return 0;
}

void motion_verify_set_active(bool active, int16_t speed) {
    is_active = active;
    expected_speed = speed;
    
    if (active) {
        last_status = MOTION_OK;
        // Sample at 20Hz (every 50ms)
        k_timer_start(&verify_timer, K_MSEC(50), K_MSEC(50));
        LOG_DBG("Motion verification activated (speed: %d)", speed);
    } else {
        k_timer_stop(&verify_timer);
        LOG_DBG("Motion verification deactivated");
    }
}

int motion_verify_deinit(void) {
    is_active = false;
    k_timer_stop(&verify_timer);
    LOG_INF("Motion verification deinitialized");
    return 0;
}
#include "robot.h"
#include "robot/calibration.h"
#include "robot/sensors/crash_detect.h"
#include "robot/sensors/int1_gpio.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/sensors/tap_detect.h"
#include "ses_assignment.h"
#include "mergebot.h"
#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(robot, LOG_LEVEL_DBG);

#define INT1_GPIO_NODE DT_NODELABEL(gpio0)
#define INT1_PIN 12

struct gpio_int_handle int1_handle;

static const struct gpio_dt_spec int1_gpio = {
    .port = DEVICE_DT_GET(INT1_GPIO_NODE),
    .pin = INT1_PIN,
    .dt_flags = GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN
};

static robot_imu_mode_t current_mode = IMU_MODE_OFF;
static K_SEM_DEFINE(crash_sem, 0, 1);
static volatile bool was_moving_forward = false;

static void default_gpio_callback(gpio_pin_t pin, void *user_data) {
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);
}

static void crash_handler(bool was_forward) {
    was_moving_forward = was_forward;
    k_sem_give(&crash_sem);
}

static void crash_recovery_routine(void) {
    LOG_WRN("CRASH DETECTED");
    
    robot_set_status(STATUS_CRASH);
    
    mb_drive(0, 0);
    
    k_sleep(K_MSEC(250));

    int16_t backup_speed = SPEED / 2;
    mb_drive(was_moving_forward ? -backup_speed : backup_speed, 
             was_moving_forward ? -backup_speed : backup_speed);
    k_sleep(K_MSEC(1000));
    
    mb_drive(0, 0);
    
    LOG_ERR("HALTED waiting for reboot");
    while (1) {
        k_sleep(K_FOREVER);
    }
}
void robot_set_imu_mode(robot_imu_mode_t new_mode) {
    if (current_mode == new_mode) return;

    switch (current_mode) {
        case IMU_MODE_TAP:
            tap_detect_deinit();
            break;
        case IMU_MODE_CRASH:
            crash_detect_deinit();
            break;
        default:
            break;
    }

    k_sleep(K_MSEC(10));

    switch (new_mode) {
        case IMU_MODE_TAP:
            if (tap_detect_init() != 0) {
                LOG_ERR("Failed to enable TAP mode");
            }
            break;
        case IMU_MODE_CRASH:
            if (crash_detect_init(crash_handler) != 0) {
                LOG_ERR("Failed to enable CRASH mode");
            }
            lsm6dsox_clear_interrupts();
            k_sem_reset(&crash_sem);
            break;
        default:
            break;
    }

    current_mode = new_mode;
}

static int robot_gpio_init(void) {
    if (!gpio_is_ready_dt(&int1_gpio)) {
        return -ENODEV;
    }

    TRY_ERR(int, gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | int1_gpio.dt_flags));
    TRY_ERR(int, gpio_int_init_dt(&int1_handle, &int1_gpio, GPIO_INT_EDGE_RISING,
                                   default_gpio_callback, NULL));
    return 0;
}

int robot_init(void) {
    TRY_ERR(int, lsm6dsox_init());
    TRY_ERR(int, robot_gpio_init());
    TRY_ERR(int, lis3mdl_init());
    TRY_ERR(int, calibration_init(CALIBRATION_RESET));
    LOG_INF("Robot ready");
    return 0;
}

static float normalize_angle_diff(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

float robot_calculate_heading(void) {
    lis3mdl_data_t mag_data;
    double x_sum = 0.0, y_sum = 0.0;

    for (int i = 0; i < 100; i++) {
        TRY_ERR(int, lis3mdl_read_mag(&mag_data));
        double x_corr = ((double)mag_data.x - (double)g_mag_offset_x) * (double)g_mag_scale_x;
        double y_corr = ((double)mag_data.y - (double)g_mag_offset_y) * (double)g_mag_scale_y;
        x_sum += x_corr;
        y_sum += y_corr;
    }

    double x_avg = x_sum / 100.0;
    double y_avg = y_sum / 100.0;
    double heading = atan2(x_avg, y_avg) * 180.0 / M_PI;
    
    return (float)heading;
}

void robot_move(int32_t distance_mm) {
    if (distance_mm == 0) return;
    
    LOG_INF("Moving %d mm", distance_mm);

    bool forward = (distance_mm > 0);
    int32_t target_distance = abs(distance_mm);
    uint32_t duration_ms = (uint32_t)((float)target_distance / MM_PER_MS_DRIVE);

    crash_detect_set_active(true, forward);
    k_sem_reset(&crash_sem);
    k_sleep(K_MSEC(50));

    int64_t start_time = k_uptime_get();
    int16_t base_speed = SPEED;

    while (true) {
        if (k_sem_take(&crash_sem, K_NO_WAIT) == 0) {
            crash_detect_set_active(false, false);
            crash_recovery_routine();
            UNREACHABLE();
        }

        int64_t elapsed = k_uptime_get() - start_time;
        if (elapsed >= duration_ms) break;

        float distance_remaining = (duration_ms - elapsed) * MM_PER_MS_DRIVE;
        int16_t current_speed = (int16_t)(distance_remaining * KP);
        
        if (current_speed > base_speed) current_speed = base_speed;
        if (current_speed < MIN_SPEED) current_speed = MIN_SPEED;

        mb_drive(forward ? current_speed : -current_speed, 
                 forward ? current_speed : -current_speed);
        k_sleep(K_MSEC(5));
    }

    crash_detect_set_active(false, false);
    mb_drive(forward ? -SPEED : SPEED, forward ? -SPEED : SPEED);
    k_sleep(K_MSEC(25));
    mb_drive(0, 0);
    k_sleep(K_MSEC(100));
    LOG_INF("Movement complete");
}

void robot_turn(int32_t angle_deg) {
    if (angle_deg == 0) return;

    float target_angle = fabsf((float)angle_deg);
    float current_angle = 0.0f;
    lsm6dsox_gyro_data_t gyro;
    int64_t last_time = k_uptime_get();
    int16_t base_speed = TURNSPEED;

    while (current_angle < target_angle) {
        if (lsm6dsox_read_gyro(&gyro) == 0) {
            int64_t now = k_uptime_get();
            if (now > last_time) {
                float dt = (now - last_time) / 1000.0f;
                last_time = now;
                float dps = lsm6dsox_gyro_to_dps(gyro.z) - g_gyro_bias_z;
                
                if (fabsf(dps) > 0.5f) {
                    current_angle += fabsf(dps * dt);
                }
            }
        }

        float error = target_angle - current_angle;
        int16_t current_speed = (int16_t)(error * KP);
        
        if (current_speed > base_speed) current_speed = base_speed;
        if (current_speed < MIN_SPEED) current_speed = MIN_SPEED;

        mb_drive(angle_deg > 0 ? current_speed : -current_speed,
                 angle_deg > 0 ? -current_speed : current_speed);
        k_sleep(K_MSEC(5));
    }

    mb_drive(angle_deg > 0 ? -TURNSPEED : TURNSPEED,
             angle_deg > 0 ? TURNSPEED : -TURNSPEED);
    k_sleep(K_MSEC(25));
    mb_drive(0, 0);
    k_sleep(K_MSEC(100));
}

void robot_turn_to_north(void) {
    LOG_INF("Turning to North");
    uint32_t start_time = k_uptime_get_32();
    int attempts = 0;

    while (true) {
        if ((k_uptime_get_32() - start_time) > TURN_TIMEOUT_MS) {
            LOG_ERR("Timeout reached");
            break;
        }
        if (attempts >= MAX_TURN_ATTEMPTS) {
            LOG_WRN("Max attempts reached");
            break;
        }

        k_sleep(K_MSEC(1000));
        float current_heading = robot_calculate_heading();
        float delta_angle = normalize_angle_diff(ROBOT_HEADING_NORTH - current_heading);

        if (fabsf(delta_angle) <= HEADING_TOLERANCE) {
            LOG_INF("Aligned to North");
            break;
        }
        int32_t turn_angle = 0;

        if (fabsf(delta_angle) <= 60.0f) {
            turn_angle = (int32_t)roundf(delta_angle * 0.5f);            
        } else { 
            turn_angle = (int32_t)roundf(delta_angle * 0.9f);
        }

        LOG_INF("Current Heading: %.2f deg, Delta: %.2f deg",
                (double)current_heading, (double)turn_angle);

        robot_turn(-turn_angle);
        k_sleep(K_MSEC(500));
        attempts++;
    }
}

void robot_set_status(robot_status_t status) {
    mb_leds_off();
    switch (status) {
        case STATUS_OK:
            mb_led_toggle(MB_LED_G);
            break;
        case STATUS_FAST:
            mb_led_toggle(MB_LED_B);
            break;
        case STATUS_SLOW:
            mb_led_toggle(MB_LED_R);
            mb_led_toggle(MB_LED_G);
            break;
        case STATUS_CRASH:
            mb_led_toggle(MB_LED_R);
            break;
    }
}
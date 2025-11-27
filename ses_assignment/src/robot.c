#include "robot.h"
#include "robot/calibration.h"
#include "robot/sensors/crash_detect.h"
#include "robot/sensors/int1_gpio.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/sensors/motion_verify.h"
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
        case IMU_MODE_MOTION_VERIFY:
            motion_verify_deinit();
            break;
        default:
            break;
    }

    k_sleep(K_MSEC(10));

    // Initialize new mode
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
        case IMU_MODE_MOTION_VERIFY:
            if (motion_verify_init() != 0) {
                LOG_ERR("Failed to enable MOTION_VERIFY mode");
            }
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
    TRY_ERR(int, motion_verify_init());
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
    
    // Calculate target ticks: (distance / circumference) * ticks_per_rev
    float circumference = M_PI * WHEEL_DIAMETER_MM;
    int32_t target_ticks = (int32_t)((target_distance / circumference) * TICKS_PER_REV);
    
    LOG_INF("Target ticks: %d", target_ticks);

    if (current_mode == IMU_MODE_MOTION_VERIFY) {
        motion_verify_start(forward);
    }
    else {
        crash_detect_set_active(true, forward);
        k_sem_reset(&crash_sem);
    }
    
    k_sleep(K_MSEC(50));

    int32_t start_left, start_right;
    mb_angle(&start_left, &start_right);
    
    int32_t current_avg_ticks = 0;

    mb_drive(forward ? SPEED_LEFT : -SPEED_LEFT, 
             forward ? SPEED_RIGHT : -SPEED_RIGHT);

    while (current_avg_ticks < target_ticks) {
        if (current_mode != IMU_MODE_MOTION_VERIFY) {
            if (k_sem_take(&crash_sem, K_NO_WAIT) == 0) {
                crash_detect_set_active(false, false);
                crash_recovery_routine();
                UNREACHABLE();
            }
        }
        int32_t current_left, current_right;
        mb_angle(&current_left, &current_right);
        
        int32_t delta_left = abs(current_left - start_left);
        int32_t delta_right = abs(current_right - start_right);
        
        current_avg_ticks = (delta_left + delta_right) / 2;        
                 
        k_sleep(K_MSEC(10));
    }

    if (current_mode == IMU_MODE_MOTION_VERIFY) {
        motion_verify_stop();
    } else {
        crash_detect_set_active(false, false);
    }
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
    int consecutive_success = 0;

    while (true) {
        if ((k_uptime_get_32() - start_time) > TURN_TIMEOUT_MS) {
            LOG_ERR("Timeout reached");
            break;
        }

        float current_heading = robot_calculate_heading();
        float error = normalize_angle_diff(ROBOT_HEADING_NORTH - current_heading);

        LOG_DBG("Heading: %.2f, Error: %.2f", (double)current_heading, (double)error);

        if (fabsf(error) <= ALIGN_TOLERANCE) {
            consecutive_success++;
            if (consecutive_success >= 3) { // Ensure stability
                LOG_INF("Aligned to North");
                break;
            }
        } else {
            consecutive_success = 0;
        }

        int16_t turn_speed = (int16_t)(error * KP_ALIGN);
        
        // Clamp speed to max TURNSPEED
        if (turn_speed > TURNSPEED) turn_speed = TURNSPEED;
        if (turn_speed < -TURNSPEED) turn_speed = -TURNSPEED;
        
        // Ensure minimum speed to overcome friction
        if (turn_speed > 0 && turn_speed < MIN_SPEED) turn_speed = MIN_SPEED;
        if (turn_speed < 0 && turn_speed > -MIN_SPEED) turn_speed = -MIN_SPEED;

        // mb_drive(-turn_speed, turn_speed);
        LOG_INF("Heading: %.2f, Error: %.2f, Speed: %d", (double)current_heading, (double)error, turn_speed);
        k_sleep(K_MSEC(50)); // Update rate
    }

    mb_drive(0, 0);
    k_sleep(K_MSEC(100));
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
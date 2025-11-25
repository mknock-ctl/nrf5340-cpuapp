#include "robot.h"
#include "robot/calibration.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/sensors/int1_gpio.h"
#include "robot/sensors/tap_detect.h"
#include "ses_assignment.h"
#include <math.h>
#include <mergebot.h>
#include "robot/sensors/crash_detect.h"
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
static void internal_crash_callback(bool moving_forward);

static void default_gpio_callback(gpio_pin_t pin, void *user_data) {
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);
}

void robot_set_imu_mode(robot_imu_mode_t new_mode) {
    if (current_mode == new_mode) return;
    k_cpu_idle();
    gpio_int_disable(&int1_handle); /* prevent race condition */

    switch (current_mode) {
        case IMU_MODE_TAP:
            tap_detect_deinit();
            break;
        case IMU_MODE_CRASH:
            crash_detect_deinit();
            break;
        case IMU_MODE_OFF:
        default:
            break;
    }

    k_busy_wait(1000);
    lsm6dsox_clear_interrupts();
    const struct device *port = int1_handle.gpio_spec.port;
    gpio_pin_t pin = int1_handle.gpio_spec.pin;

    gpio_pin_interrupt_configure(port, pin, GPIO_INT_DISABLE);
    k_busy_wait(100);
    switch (new_mode) {
        case IMU_MODE_TAP:
            if (tap_detect_init() != 0) {
                LOG_ERR("Failed to enable TAP mode");
            }
            break;
        case IMU_MODE_CRASH:
            if (crash_detect_init(internal_crash_callback) != 0) {
                LOG_ERR("Failed to enable CRASH mode");
            }
            crash_detect_set_active(true, true);
            break;
        case IMU_MODE_OFF:
        default:
            break;
    }
    lsm6dsox_clear_interrupts();

    gpio_int_enable(&int1_handle);
    current_mode = new_mode;
    LOG_INF("IMU Mode Switched to: %d", new_mode);
}

static int robot_gpio_init(void) {
    if (!gpio_is_ready_dt(&int1_gpio)) {
        return -ENODEV;
    }

    TRY_ERR(int, gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | int1_gpio.dt_flags));
    TRY_ERR(int, gpio_int_init_dt(&int1_handle, 
                               &int1_gpio, 
                               GPIO_INT_EDGE_RISING, 
                               default_gpio_callback, 
                               NULL));
    return 0;
}

int robot_init(void){
    TRY_ERR(int, lsm6dsox_init());
    TRY_ERR(int, robot_gpio_init());
    TRY_ERR(int, lis3mdl_init());
    TRY_ERR(int, calibration_init(CALIBRATION_RESET));

    LOG_INF("Robot ready");
    return 0;
}

static void internal_crash_callback(bool was_forward) {
    mb_drive(0, 0);
    k_sleep(K_MSEC(CRASH_WAIT_MS));

    int16_t backup_speed = SPEED / 2;
    if (was_forward) {
        mb_drive(-backup_speed, -backup_speed);
    } else {
        mb_drive(backup_speed, backup_speed);
    }
    k_sleep(K_MSEC(CRASH_BACKUP_TIME_MS));

    mb_drive(0, 0);

    robot_set_status(STATUS_CRASH);

    while (1) {
        k_sleep(K_FOREVER);
    }
}

static float normalize_angle_diff(float angle) {
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

float robot_calculate_heading(void) {
    lis3mdl_data_t mag_data;
    
    double x_sum = 0.0;
    double y_sum = 0.0;
    
    // TODO: use exponential moving average or calibration to deal with the randomness, this is really inefficient
    for (int i = 0; i < 100; i++) {
        TRY_ERR(int, lis3mdl_read_mag(&mag_data));
        
        int32_t x_raw = mag_data.x;
        int32_t y_raw = mag_data.y;
        
        double x_corr = ((double)x_raw - (double)g_mag_offset_x) * (double)g_mag_scale_x;
        double y_corr = ((double)y_raw - (double)g_mag_offset_y) * (double)g_mag_scale_y;
        
        x_sum += x_corr;
        y_sum += y_corr;
    }
    
    double x_corr_counts = x_sum / 100.0;
    double y_corr_counts = y_sum / 100.0;
    
    double angle_rad = 30.0 * M_PI / 180.0;
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);
    
    double x_rotated = x_corr_counts * cos_a - y_corr_counts * sin_a;
    double y_rotated = x_corr_counts * sin_a + y_corr_counts * cos_a;
    
    //double heading = atan2(y_rotated, x_rotated) * 180.0 / M_PI; // degrees
    double heading = atan2(y_corr_counts, x_corr_counts) * 180.0 / M_PI;
    float heading_deg = normalize_angle_diff((float)heading);
    
    LOG_DBG("Heading: %.1f (x:%.3f y:%.3f, rotated x:%.3f y:%.3f)",
            (double)heading_deg, x_corr_counts, y_corr_counts, x_rotated, y_rotated);
    
    return heading_deg;
}

void robot_move(int32_t distance_mm) {
    if (distance_mm == 0)
        return;

    bool forward = (distance_mm > 0);
    int32_t target_distance = abs(distance_mm);
    uint32_t duration_ms = (uint32_t)((float)target_distance / MM_PER_MS_DRIVE);

    crash_detect_set_active(true, forward);

    int64_t start_time = k_uptime_get();
    int16_t base_speed = SPEED;
    int64_t elapsed = 0;

    while (elapsed < duration_ms) {
        elapsed = k_uptime_get() - start_time;
        
        float time_remaining = duration_ms - elapsed;
        float distance_remaining = time_remaining * MM_PER_MS_DRIVE;
        
        int16_t current_speed = (int16_t)(distance_remaining * KP);
        
        if (current_speed > base_speed)
            current_speed = base_speed;
        if (current_speed < MIN_SPEED)
            current_speed = MIN_SPEED;
        
        if (forward) {
            mb_drive(current_speed, current_speed);
        } else {
            mb_drive(-current_speed, -current_speed);
        }
        
        k_sleep(K_MSEC(5));
    }

    crash_detect_set_active(false, false);
    
    if (forward) {
        mb_drive(-SPEED, -SPEED);
    } else {
        mb_drive(SPEED, SPEED);
    }
    k_sleep(K_MSEC(25));
    
    mb_drive(0, 0);
    k_sleep(K_MSEC(100));
    
    LOG_INF("Moved %.1f mm (Target %d)", (double)elapsed, distance_mm);
}

void robot_turn(int32_t angle_deg) {
    if (angle_deg == 0)
        return;

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

        if (current_speed > base_speed)
            current_speed = base_speed;
        if (current_speed < MIN_SPEED)
            current_speed = MIN_SPEED;

        if (angle_deg > 0) {
            mb_drive(current_speed, -current_speed);
        } else {
            mb_drive(-current_speed, current_speed);
        }

        k_sleep(K_MSEC(5));
    }

    if (angle_deg > 0) {
        mb_drive(-TURNSPEED, TURNSPEED); // reverse
    } else {
        mb_drive(TURNSPEED, -TURNSPEED);
    }
    k_sleep(K_MSEC(25)); // Brake duration (needs tuning)

    mb_drive(0, 0);

    k_sleep(K_MSEC(100));

    LOG_INF("Turned %.1f deg (Target %d)", (double)current_angle, angle_deg);
}

void robot_turn_to_north(void) {
    float current_heading, delta_angle;
    int32_t turn_angle;
    uint32_t start_time = k_uptime_get_32();
    int attempts = 0;

    LOG_INF("Navigating to the north");

    while (true) {
        if ((k_uptime_get_32() - start_time) > TURN_TIMEOUT_MS) {
            LOG_ERR("Timeout reached");
            break;
        }

        if (attempts >= MAX_TURN_ATTEMPTS) {
            LOG_WRN("Max attempts reached");
            break;
        }

        current_heading = robot_calculate_heading();

        delta_angle = ROBOT_HEADING_NORTH - current_heading;
        delta_angle = normalize_angle_diff(delta_angle);

        if (fabsf(delta_angle) <= HEADING_TOLERANCE) {
            LOG_INF("Aligned to North: %.1f", (double)current_heading);
            break;
        }

        turn_angle = (int32_t)roundf(delta_angle);

        LOG_INF("Heading: %.1f -> Turning %d", (double)current_heading, turn_angle);

        robot_turn(turn_angle);

        k_sleep(K_MSEC(500)); // allow magnetometer to settle
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
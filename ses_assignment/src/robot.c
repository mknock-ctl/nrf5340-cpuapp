#include "robot.h"
#include "robot/calibration.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "ses_assignment.h"
#include <math.h>
#include <mergebot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(robot, LOG_LEVEL_DBG);

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
    
    double heading = atan2(y_rotated, x_rotated) * 180.0 / M_PI; // degrees
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
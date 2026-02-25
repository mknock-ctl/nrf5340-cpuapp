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
static robot_status_t current_robot_status = STATUS_OK;
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
            motion_verify_deinit();
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
            motion_verify_init();
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

static void mb_drive_compensated(int16_t left_speed, int16_t right_speed) {
    // Apply scale factors from calibration
    int16_t compensated_left = (int16_t)(left_speed * g_motor_left_scale);
    int16_t compensated_right = (int16_t)(right_speed * g_motor_right_scale);
    
    mb_drive(compensated_left, compensated_right);
}
    
void robot_move(int32_t distance_mm) {
    robot_move_with_factor(distance_mm, 4);
}

//target_ticks = (int32_t)(abs_distance * g_ticks_per_mm);
void robot_move_with_factor(int32_t distance_mm, int32_t ramp_factor) {
    if (distance_mm == 0) return;
    if (!ramp_factor) ramp_factor = 4;
    
    bool forward = (distance_mm > 0);
    int32_t abs_distance = abs(distance_mm);
    
    // Calculate target ticks
    float circumference = M_PI * WHEEL_DIAMETER_MM;
    int32_t target_ticks = (int32_t)((abs_distance / circumference) * TICKS_PER_REV);

    // Setup crash detection
    lsm6dsox_enable_crash_and_motion();
    crash_detect_set_active(true, forward);
    motion_verify_start(forward);
    k_sem_reset(&crash_sem);
    k_sleep(K_MSEC(50));

    // Get starting positions
    int32_t start_left, start_right;
    mb_angle(&start_left, &start_right);

    const float KP = 0.1f;      // Low for gentler response
    const float KI = 0.0008f;     // Lower to prevent integral windup
    const float KD = 0.05f;      // Derivative causes jerk
    const int16_t MAX_CORRECTION = 40;
    
    const int32_t RAMP_UP_TICKS = target_ticks / ramp_factor;
    const int32_t RAMP_DOWN_TICKS = target_ticks / ramp_factor;
    const int16_t MIN_RAMP_SPEED = SPEED / 3;
    
    int16_t base_speed = SPEED;
    
    float integral = 0.0f;
    float last_error = 0.0f;
    
    uint32_t last_time = k_uptime_get_32();
    int32_t current_avg_ticks = 0;
    int loop_count = 0;

    while (true) {
        loop_count++;
        
        // Check for crash
        if (current_mode != IMU_MODE_MOTION_VERIFY) {
            if (k_sem_take(&crash_sem, K_NO_WAIT) == 0) {
                crash_detect_set_active(false, false);
                crash_recovery_routine();
                UNREACHABLE();
            }
        }

        uint32_t now = k_uptime_get_32();
        float dt = (now - last_time) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f;
        last_time = now;

        int32_t curr_l, curr_r;
        mb_angle(&curr_l, &curr_r);
        int32_t delta_left = abs(curr_l - start_left);
        int32_t delta_right = abs(curr_r - start_right);
        
        current_avg_ticks = (delta_left + delta_right) / 2;
        int32_t remaining_ticks = target_ticks - current_avg_ticks;
        
        if (current_avg_ticks >= target_ticks) break;
        if (current_avg_ticks > target_ticks + 100) break;
        if (remaining_ticks <= 0) break;
        if (loop_count > 15000) break;
        
        int32_t encoder_diff = delta_left - delta_right;
        float error = (float)encoder_diff;
        
        int16_t current_base_speed;
        if (current_avg_ticks < RAMP_UP_TICKS) {
            float ramp_factor = (float)current_avg_ticks / (float)RAMP_UP_TICKS;
            current_base_speed = (int16_t)(MIN_RAMP_SPEED + 
                                  (base_speed - MIN_RAMP_SPEED) * ramp_factor);
        } else if (remaining_ticks < RAMP_DOWN_TICKS) {
            float ramp_factor = (float)remaining_ticks / (float)RAMP_DOWN_TICKS;
            current_base_speed = (int16_t)(MIN_RAMP_SPEED + 
                                  (base_speed - MIN_RAMP_SPEED) * ramp_factor);
            if (current_base_speed < MIN_SPEED) current_base_speed = MIN_SPEED;
        } else {
            current_base_speed = base_speed;
        }
        
        // Simple PID
        float p_term = KP * error;
        
        integral += error * dt;
        const float INTEGRAL_MAX = 300.0f;
        if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
        if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;
        
        float d_term = KD * (error - last_error) / dt;
        last_error = error;
        
        // PID output
        float pid_output = p_term + (KI * integral) + d_term;
        int16_t correction = (int16_t)pid_output;
        
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
        
        int16_t left_cmd = current_base_speed - correction;
        int16_t right_cmd = current_base_speed + correction;
        
        if (!forward) {
            left_cmd = -left_cmd;
            right_cmd = -right_cmd;
        }
        
        float left_final = left_cmd * g_motor_left_scale;
        float right_final = right_cmd * g_motor_right_scale;
        
        int16_t left_final_cmd = (int16_t)left_final;
        int16_t right_final_cmd = (int16_t)right_final;
        
        const int16_t SPEED_LIMIT = 380;
        
        if (abs(left_final_cmd) > SPEED_LIMIT || abs(right_final_cmd) > SPEED_LIMIT) {
            float max_val = fmaxf(abs(left_final_cmd), abs(right_final_cmd));
            float scale = SPEED_LIMIT / max_val;
            
            left_final_cmd = (int16_t)(left_final_cmd * scale);
            right_final_cmd = (int16_t)(right_final_cmd * scale);
            
            integral *= 0.7f;
        }
        
        mb_drive(left_final_cmd, right_final_cmd);
        
        k_sleep(K_MSEC(10));
    }

    motion_verify_stop();
    crash_detect_set_active(false, false);
    
    int16_t brake_speed = base_speed / 2;
    if (forward) {
        mb_drive((int16_t)(-brake_speed * g_motor_left_scale), 
                 (int16_t)(-brake_speed * g_motor_right_scale));
    } else {
        mb_drive((int16_t)(brake_speed * g_motor_left_scale), 
                 (int16_t)(brake_speed * g_motor_right_scale));
    }
    k_sleep(K_MSEC(30));
    
    mb_drive(0, 0);
    k_sleep(K_MSEC(100));
}

void robot_turn(int32_t angle_deg) {
    if (angle_deg == 0) return;

    LOG_INF("Turning %d degrees %s", angle_deg, angle_deg > 0 ? "CW" : "CCW");

    mb_drive(0, 0);
    k_sleep(K_MSEC(300));

    const float KP = 4.5f;
    const float KI = 0.15f;
    const float KD = 2.5f;
    const int16_t MAX_SPEED = TURNSPEED;
    const float GYRO_TOLERANCE = 0.3f;
    const float GYRO_THRESHOLD = 0.12f;
    const float INTEGRAL_LIMIT = 15.0f;
    const float GYRO_FILTER_ALPHA = 0.85f;
    
    bool is_clockwise = (angle_deg > 0);
    
    float turn_scale = is_clockwise ? g_turn_cw_scale : g_turn_ccw_scale;
    float stop_ahead = is_clockwise ? g_cw_stop_ahead : g_ccw_stop_ahead;
    float brake_scale = is_clockwise ? g_cw_brake_scale : g_ccw_brake_scale;
    
    float target_angle = fabsf((float)angle_deg);
    float gyro_angle = 0.0f;
    float integral = 0.0f;
    float filtered_dps = 0.0f;

    float error = target_angle - gyro_angle;
    float last_error = error;
    int64_t last_time = k_uptime_get();

    int gyro_stable_count = 0;
    int loop_count = 0;    
    while (true) {
        loop_count++;
        lsm6dsox_gyro_data_t gyro;
        int64_t now = k_uptime_get();
        float dt = (now > last_time) ? ((now - last_time) / 1000.0f) : 0.001f;
        last_time = now;
        
        if (lsm6dsox_read_gyro(&gyro) == 0) {
            float raw_dps = lsm6dsox_gyro_to_dps(gyro.z);
            filtered_dps = GYRO_FILTER_ALPHA * raw_dps + 
                                        (1.0f - GYRO_FILTER_ALPHA) * filtered_dps;
            
            if (fabsf(filtered_dps) > GYRO_THRESHOLD) {
                gyro_angle += filtered_dps * dt;
            }
        }
        
        float abs_gyro_angle = fabsf(gyro_angle);
        error = target_angle - abs_gyro_angle;
        
        if (error <= stop_ahead && fabsf(filtered_dps) < 5.0f) {
            LOG_INF("Pre-emptive stop at %.2f degrees (stop_ahead=%.2f)", 
                   (double)gyro_angle, (double)stop_ahead);
            break;
        }
        
        if (fabsf(error) > 1.0f) {
            integral += error * dt * 0.05f;
            if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
            if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
        } else {
            integral *= 0.9f;
        }
        
        float derivative = (dt > 0.0f) ? ((error - last_error) / dt) : 0.0f;
        last_error = error;
        
        // Check stability
        if (fabsf(error) <= GYRO_TOLERANCE && fabsf(filtered_dps) < GYRO_THRESHOLD) {
            gyro_stable_count++;
            if (gyro_stable_count >= 8) {
                LOG_INF("Turn complete: %.2f degrees (error: %.2f)", 
                       (double)gyro_angle, (double)error);
                break;
            }
        } else {
            gyro_stable_count = 0;
        }
        
        if (abs_gyro_angle > (target_angle + 0.5f)) {
            LOG_WRN("Overshoot detected: %.2f degrees", (double)-error);
            break;
        }
        
        if (loop_count > 3000) {
            LOG_WRN("Turn timeout");
            break;
        }
        
        // PID control (signed control only cares about magnitude since drive uses sign separately)
        float control = (KP * error) + (KI * integral) + (KD * derivative);
        
        if (fabsf(filtered_dps) > 30.0f && error < 10.0f) {
            control *= 0.7f;
        }
        
        int16_t speed = (int16_t)fabsf(control);
        if (speed > MAX_SPEED) speed = MAX_SPEED;

        if (fabsf(error) <= 1.0f) {
            speed = MIN_SPEED;
        } else if (fabsf(error) < 8.0f) {
            float frac = (fabsf(error) - 1.0f) / 7.0f;
            speed = MIN_SPEED + (int16_t)((MAX_SPEED * 0.5f - MIN_SPEED) * frac);
        } else {
            speed = (int16_t)(MAX_SPEED * 0.7f);
        }
        
        speed = (int16_t)(speed * turn_scale);
        
        if (is_clockwise) {
            mb_drive_compensated(-speed, speed);
        } else {
            mb_drive_compensated(speed, -speed);
        }
        
        k_sleep(K_MSEC(5));
    }
    
    mb_drive(0, 0);
    k_sleep(K_MSEC(30));
    
    int16_t brake_speed;
    if (fabsf(filtered_dps) > 20.0f) {
        brake_speed = (int16_t)(MAX_SPEED / 2 * brake_scale);
    } else if (fabsf(filtered_dps) > 10.0f) {
        brake_speed = (int16_t)(MAX_SPEED / 3 * brake_scale);
    } else {
        brake_speed = (int16_t)(MAX_SPEED / 4 * brake_scale);
    }
    
    if (is_clockwise) {
        mb_drive_compensated(brake_speed, -brake_speed);
    } else {
        mb_drive_compensated(-brake_speed, brake_speed);
    }
    k_sleep(K_MSEC(25));
    
    mb_drive(0, 0);
    k_sleep(K_MSEC(300));
}

void robot_turn_to_north(void) {
    LOG_INF("Turning to North");
    uint32_t start_time = k_uptime_get_32();
    
    const float tolerance = 8.0f; // Increased tolerance for noisy sensor
    int consecutive_good = 0;
    const int required_good = 3;
    
    float last_error = 0.0f;
    int oscillation_count = 0;
    
    mb_drive(0, 0);
    k_sleep(K_MSEC(300)); // Longer settling time
    float initial_heading = robot_calculate_heading();
    float target_heading = ROBOT_HEADING_NORTH;
    
    LOG_INF("Initial heading: %.2f, Target: %.2f", (double)initial_heading, (double)target_heading);

    float initial_error = normalize_angle_diff(target_heading - initial_heading);
    bool turn_clockwise = (initial_error < 0); // negative error means turn counter-clockwise (left)
    
    LOG_INF("Initial error: %.2f, will turn %s", (double)initial_error, 
            turn_clockwise ? "clockwise" : "counter-clockwise");

    while (true) {
        if ((k_uptime_get_32() - start_time) > TURN_TIMEOUT_MS) {
            LOG_ERR("Timeout reached");
            mb_drive(0, 0);
            break;
        }

        // Read heading with motors OFF
        mb_drive(0, 0);
        k_sleep(K_MSEC(300));
        
        float current_heading = robot_calculate_heading();
        float error = normalize_angle_diff(target_heading - current_heading);
        float abs_error = fabsf(error);

        LOG_INF("Heading: %.2f, Error: %.2f", (double)current_heading, (double)error);

        if (abs_error <= tolerance) {
            consecutive_good++;
            LOG_INF("Good reading %d/%d (error: %.2f)", consecutive_good, required_good, (double)abs_error);
            
            if (consecutive_good >= required_good) {
                LOG_INF("Aligned to North (heading: %.2f)", (double)current_heading);
                mb_drive(0, 0);
                break;
            }
            continue;
        } else {
            consecutive_good = 0;
        }

        if (last_error != 0.0f && 
            ((last_error > 0 && error < 0) || (last_error < 0 && error > 0)) &&
            abs_error > tolerance) {
            oscillation_count++;
            LOG_WRN("Oscillation detected (%d), error %.2f -> %.2f", 
                    oscillation_count, (double)last_error, (double)error);
        } else {
            oscillation_count = 0;
        }
        last_error = error;

        if (oscillation_count >= 2) {
            LOG_INF("High oscillation detected, waiting for better reading...");
            k_sleep(K_MSEC(200));
            current_heading = robot_calculate_heading();
            error = normalize_angle_diff(target_heading - current_heading);
            abs_error = fabsf(error);
            LOG_INF("After wait - Heading: %.2f, Error: %.2f", 
                    (double)current_heading, (double)error);
            oscillation_count = 0;
        }

        int16_t turn_speed;
        int32_t turn_duration_ms;
        
        float speed_multiplier = (oscillation_count > 0) ? 0.7f : 1.0f;
        
        if (abs_error > 90.0f) {
            turn_speed = TURNSPEED * speed_multiplier;
            turn_duration_ms = 300;
        } else if (abs_error > 45.0f) {
            turn_speed = TURNSPEED * speed_multiplier;
            turn_duration_ms = 200;
        } else if (abs_error > 20.0f) {
            turn_speed = TURNSPEED * 0.8f * speed_multiplier;
            turn_duration_ms = 120;
        } else if (abs_error > 12.0f) {
            turn_speed = TURNSPEED * 0.6f * speed_multiplier;
            turn_duration_ms = 80;
        } else {
            turn_speed = MIN_SPEED;
            turn_duration_ms = 40;
        }

        bool current_turn_clockwise = (error < 0);
        
        if (current_turn_clockwise) {
            mb_drive(turn_speed, -turn_speed);
        } else {
            mb_drive(-turn_speed, turn_speed);
        }
        
        LOG_INF("Turning %s at speed %d for %d ms", 
                current_turn_clockwise ? "CW" : "CCW", turn_speed, turn_duration_ms);
        
        k_sleep(K_MSEC(turn_duration_ms));
        
        mb_drive(0, 0);
        k_sleep(K_MSEC(50));
        
        int16_t brake_speed = turn_speed / 3;
        if (current_turn_clockwise) {
            mb_drive(-brake_speed, brake_speed);
        } else {
            mb_drive(brake_speed, -brake_speed);
        }
        k_sleep(K_MSEC(30));
        mb_drive(0, 0);
    }

    mb_drive(0, 0);
    k_sleep(K_MSEC(300));
}

void robot_set_status(robot_status_t status) {
    mb_leds_off();

    if (current_robot_status == STATUS_CRASH) {
        mb_led_toggle(MB_LED_R);
        return;
    }

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
    current_robot_status = status;
}
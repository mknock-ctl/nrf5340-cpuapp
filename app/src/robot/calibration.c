#include "robot/calibration.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/settings.h"
#include "ses_assignment.h"
#include "mergebot.h"
#include <errno.h>
#include <stdio.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(calibration, LOG_LEVEL_DBG);

// IMU calibration
float g_gyro_bias_z = 0.0f;
float g_mag_offset_x = -1.0f;
float g_mag_offset_y = -1.0f;
float g_mag_scale_x = -1.0f;
float g_mag_scale_y = -1.0f;
float g_accel_bias_x = 0.0f;
float g_ticks_per_mm = -1.0f;

float g_motor_left_scale = 1.0f;
float g_motor_right_scale = 1.0f;
float g_turn_cw_scale = 1.0f;
float g_turn_ccw_scale = 1.0f;
float g_cw_stop_ahead = 0.7f;
float g_ccw_stop_ahead = 1.0f;
float g_cw_brake_scale = 1.0f;
float g_ccw_brake_scale = 1.0f;

static const setting_entry_t robot_settings[] = {
    {"bias_z", &g_gyro_bias_z, sizeof(g_gyro_bias_z)},
    {"mag_offset_x", &g_mag_offset_x, sizeof(g_mag_offset_x)},
    {"mag_offset_y", &g_mag_offset_y, sizeof(g_mag_offset_y)},
    {"mag_scale_x", &g_mag_scale_x, sizeof(g_mag_scale_x)},
    {"mag_scale_y", &g_mag_scale_y, sizeof(g_mag_scale_y)},
    {"accel_bias_x", &g_accel_bias_x, sizeof(g_accel_bias_x)},
    {"ticks_per_mm", &g_ticks_per_mm, sizeof(g_ticks_per_mm)},

    {"motor_left_scale", &g_motor_left_scale, sizeof(g_motor_left_scale)},
    {"motor_right_scale", &g_motor_right_scale, sizeof(g_motor_right_scale)},
    {"turn_cw_scale", &g_turn_cw_scale, sizeof(g_turn_cw_scale)},
    {"turn_ccw_scale", &g_turn_ccw_scale, sizeof(g_turn_ccw_scale)},
    {"cw_stop_ahead", &g_cw_stop_ahead, sizeof(g_cw_stop_ahead)},
    {"ccw_stop_ahead", &g_ccw_stop_ahead, sizeof(g_ccw_stop_ahead)},
    {"cw_brake_scale", &g_cw_brake_scale, sizeof(g_cw_brake_scale)},
    {"ccw_brake_scale", &g_ccw_brake_scale, sizeof(g_ccw_brake_scale)}
};

#define MAX_SAMPLES 2000
static int16_t samples_x[MAX_SAMPLES];
static int16_t samples_y[MAX_SAMPLES];

int calibration_init(bool reset) {
    TRY_ERR(int, settings_init_and_register("calibration", robot_settings, ARRAY_SIZE(robot_settings)));

    if (reset) {
        const float zero = 0.0f;
        const float neg = -1.0f;
        const float one = 1.0f;
        const float cw_default = 0.7f;
        const float ccw_default = 1.0f;
        
        // IMU
        settings_save_float("calibration/bias_z", zero);
        settings_save_float("calibration/mag_offset_x", neg);
        settings_save_float("calibration/mag_offset_y", neg);
        settings_save_float("calibration/mag_scale_x", neg);
        settings_save_float("calibration/mag_scale_y", neg);
        settings_save_float("calibration/accel_bias_x", zero);
        settings_save_float("calibration/ticks_per_mm", neg);
        
        // Motor asymmetry
        settings_save_float("calibration/motor_left_scale", one);
        settings_save_float("calibration/motor_right_scale", one);
        settings_save_float("calibration/turn_cw_scale", one);
        settings_save_float("calibration/turn_ccw_scale", one);
        settings_save_float("calibration/cw_stop_ahead", cw_default);
        settings_save_float("calibration/ccw_stop_ahead", ccw_default);
        settings_save_float("calibration/cw_brake_scale", one);
        settings_save_float("calibration/ccw_brake_scale", one);
        
        g_gyro_bias_z = 0.0f;
        g_mag_offset_x = -1.0f;
        g_mag_offset_y = -1.0f;
        g_mag_scale_x = -1.0f;
        g_mag_scale_y = -1.0f;
        g_accel_bias_x = 0.0f;
        g_ticks_per_mm = -1.0f;
        g_motor_left_scale = 1.0f;
        g_motor_right_scale = 1.0f;
        g_turn_cw_scale = 1.0f;
        g_turn_ccw_scale = 1.0f;
        g_cw_stop_ahead = 0.7f;
        g_ccw_stop_ahead = 1.0f;
        g_cw_brake_scale = 1.0f;
        g_ccw_brake_scale = 1.0f;
    }

    LOG_INF("Gyro bias: %.3f dps", (double)g_gyro_bias_z);
    LOG_INF("Mag: offset X=%.1f Y=%.1f, scale X=%.3f Y=%.3f", 
            (double)g_mag_offset_x, (double)g_mag_offset_y,
            (double)g_mag_scale_x, (double)g_mag_scale_y);
    LOG_INF("Accel bias X: %.1f", (double)g_accel_bias_x);
    LOG_INF("Encoder: %.4f ticks/mm", (double)g_ticks_per_mm);
    LOG_INF("Motor scales: L=%.3f R=%.3f", (double)g_motor_left_scale, (double)g_motor_right_scale);
    LOG_INF("Turn scales: CW=%.3f CCW=%.3f", (double)g_turn_cw_scale, (double)g_turn_ccw_scale);
    LOG_INF("Stop ahead: CW=%.2f° CCW=%.2f°", (double)g_cw_stop_ahead, (double)g_ccw_stop_ahead);
    LOG_INF("Brake scales: CW=%.2f CCW=%.2f", (double)g_cw_brake_scale, (double)g_ccw_brake_scale);
    return 0;
}

bool calibration_needed(void) {
    return (g_mag_offset_x < 0 || g_mag_offset_y < 0 ||
            g_mag_scale_x < 0 || g_mag_scale_y < 0 ||
            fabsf(g_accel_bias_x) < 0.001f ||
            fabsf(g_gyro_bias_z) < 0.001f ||
            g_ticks_per_mm < 0);
}

static void cal_gyro(void) {
    lsm6dsox_gyro_data_t g;
    
    // Stationary calibration
    float sum_stationary = 0.0f;
    int valid_samples = 0;
    const int target_samples = 200;
    
    k_sleep(K_MSEC(1000));
    
    for (int i = 0; i < target_samples; i++) {
        if (lsm6dsox_read_gyro(&g) == 0) {
            float dps = lsm6dsox_gyro_to_dps(g.z);
            sum_stationary += dps;
            valid_samples++;
        }
        k_sleep(K_MSEC(10));
    }
    
    float bias_stationary = sum_stationary / valid_samples;
    
    // Slow cw rotation calibration
    k_sleep(K_MSEC(1000));
    
    float sum_cw = 0.0f;
    int samples_cw = 0;
    
    mb_drive(-(MIN_SPEED + 10), (MIN_SPEED + 10));
    k_sleep(K_MSEC(500));  // Let it stabilize
    
    for (int i = 0; i < 100; i++) {
        if (lsm6dsox_read_gyro(&g) == 0) {
            float dps = lsm6dsox_gyro_to_dps(g.z);
            sum_cw += dps;
            samples_cw++;
        }
        k_sleep(K_MSEC(10));
    }
    mb_drive(0, 0);
    k_sleep(K_MSEC(500));
    
    float avg_cw = sum_cw / samples_cw;
    
    // Slow CCW rotation
    k_sleep(K_MSEC(1000));
    
    float sum_ccw = 0.0f;
    int samples_ccw = 0;
    
    mb_drive((MIN_SPEED + 10), -(MIN_SPEED + 10));
    k_sleep(K_MSEC(500));  // Let it stabilize
    
    for (int i = 0; i < 100; i++) {
        if (lsm6dsox_read_gyro(&g) == 0) {
            float dps = lsm6dsox_gyro_to_dps(g.z);
            sum_ccw += dps;
            samples_ccw++;
        }
        k_sleep(K_MSEC(10));
    }
    mb_drive(0, 0);
    k_sleep(K_MSEC(500));
    
    float avg_ccw = sum_ccw / samples_ccw;
    LOG_INF("CCW rotation average: %.3f dps", (double)avg_ccw);
    
    // Analysis
    float motion_midpoint = (avg_cw + avg_ccw) / 2.0f;
    g_gyro_bias_z = bias_stationary;
    
    float cw_offset = fabsf(avg_cw + fabsf(bias_stationary));
    float ccw_offset = fabsf(avg_ccw - fabsf(bias_stationary));
    float asymmetry = fabsf(cw_offset - ccw_offset);
    
    if (asymmetry > 5.0f) {
        LOG_WRN("Gyro shows directional asymmetry: %.2f dps difference", (double)asymmetry);
    }
    
    settings_save_float("calibration/bias_z", g_gyro_bias_z);
}

static void cal_accel(void) {
    int16_t x, y, z;
    int32_t sum = 0;
    int valid_samples = 0;
    const int target_samples = 150;
    
    k_sleep(K_MSEC(500));

    for (int i = 0; i < target_samples; i++) {
        if (lsm6dsox_read_accel_raw(&x, &y, &z) == 0) {
            sum += x;
            valid_samples++;
        }
        k_sleep(K_MSEC(10));
    }
    
    if (valid_samples < target_samples / 2) {
        return;
    }

    g_accel_bias_x = (float)sum / valid_samples;
    settings_save_float("calibration/accel_bias_x", g_accel_bias_x);
}

void cal_encoders(uint32_t duration_ms, void (*drv)(int16_t, int16_t), void (*led)(int)) {
    k_sleep(K_MSEC(2000));
    
    const int num_tests = 3;
    float results[num_tests];
    
    for (int test = 0; test < num_tests; test++) {
        if (led) led(test);
        
        k_sleep(K_MSEC(2000));
        
        int32_t start_left, start_right;
        mb_angle(&start_left, &start_right);
        
        drv(SPEED, SPEED + 4);
        k_sleep(K_MSEC(duration_ms));
        drv(0, 0);
        
        int32_t end_left, end_right;
        mb_angle(&end_left, &end_right);
        
        int32_t ticks_left = abs(end_left - start_left);
        int32_t ticks_right = abs(end_right - start_right);
        int32_t avg_ticks = (ticks_left + ticks_right) / 2;
        
        results[test] = (float)avg_ticks;
        
        LOG_INF("Test %d: Left=%d, Right=%d, Avg=%d ticks", 
                test + 1, ticks_left, ticks_right, avg_ticks);
        
        k_sleep(K_MSEC(2000));
    }
    
    float total_ticks = 0.0f;
    for (int i = 0; i < num_tests; i++) {
        total_ticks += results[i];
    }
    float avg_ticks = total_ticks / num_tests;
    
    float max_diff = 0.0f;
    for (int i = 0; i < num_tests; i++) {
        float diff = fabsf(results[i] - avg_ticks);
        if (diff > max_diff) max_diff = diff;
    }
    
    if (max_diff > avg_ticks * 0.1f) {
        LOG_WRN("High variation in encoder tests (%.1f%%) - results may be unreliable",
                (double)(max_diff / avg_ticks * 100.0f));
    }
    
    LOG_INF("Average ticks per test: %.1f", (double)avg_ticks);
    LOG_INF(">>> MEASURE THE ACTUAL DISTANCE TRAVELED <<<");
    LOG_INF(">>> Then calculate: ticks_per_mm = %.1f / actual_distance_mm", (double)avg_ticks);
    LOG_INF("");
    LOG_INF("Example: If robot traveled 500mm:");
    LOG_INF("  ticks_per_mm = %.1f / 500 = %.4f", (double)avg_ticks, (double)(avg_ticks / 500.0f));
    LOG_INF("");
    
    // Theoretical: (avg_ticks / (WHEEL_DIAMETER_MM * PI)) gives ticks per mm
    //float theoretical = avg_ticks / (M_PI * WHEEL_DIAMETER_MM);
    g_ticks_per_mm = avg_ticks / 480.0f;
    settings_save_float("calibration/ticks_per_mm", g_ticks_per_mm);
}

static void mag_seq(uint32_t dur, void (*drv)(int16_t, int16_t), void (*led)(int)) {
    int16_t min_x = 32000, max_x = -32000;
    int16_t min_y = 32000, max_y = -32000;
    lis3mdl_data_t d;
    int sample_count = 0;
    
    int stored_samples = 0;
    k_sleep(K_MSEC(2000));

    if (led) led(1);
    
    // Use VERY slow rotation to minimize motor interference
    int16_t turn_speed = MIN_SPEED + 15;
    LOG_INF("Starting slow rotation at speed %d", turn_speed);
    
    drv(turn_speed, -turn_speed); // I want at least 2 full rotations
    
    uint32_t t0 = k_uptime_get_32();
    uint32_t last_report = t0;

    while (k_uptime_get_32() - t0 < dur) {
        if (lis3mdl_read_mag(&d) == 0) {
            if (d.x < min_x) min_x = d.x;
            if (d.x > max_x) max_x = d.x;
            if (d.y < min_y) min_y = d.y;
            if (d.y > max_y) max_y = d.y;
            
            if (stored_samples < MAX_SAMPLES) {
                samples_x[stored_samples] = d.x;
                samples_y[stored_samples] = d.y;
                stored_samples++;
            }
            
            sample_count++;
        }
        
        uint32_t now = k_uptime_get_32();
        if (now - last_report > 5000) {
            LOG_INF("Progress: %d samples, X[%d,%d] Y[%d,%d]", 
                    sample_count, min_x, max_x, min_y, max_y);
            last_report = now;
        }
        
        k_sleep(K_MSEC(20));
    }

    drv(0, 0);
    if (led) led(0);
    k_sleep(K_MSEC(500));
    g_mag_offset_x = (min_x + max_x) * 0.5f;
    g_mag_offset_y = (min_y + max_y) * 0.5f;

    float range_x = (max_x - min_x) * 0.5f;
    float range_y = (max_y - min_y) * 0.5f;
    
    if (range_x < 500.0f || range_y < 500.0f) {
        g_mag_scale_x = 1.0f;
        g_mag_scale_y = 1.0f;
        return;
    }
    
    float avg_range = (range_x + range_y) * 0.5f;
    g_mag_scale_x = avg_range / range_x;
    g_mag_scale_y = avg_range / range_y;
    
    float ratio = g_mag_scale_x / g_mag_scale_y;
    if (ratio > 1.5f || ratio < 0.67f) {
        LOG_WRN("WARNING: Uneven scale ratio (%.2f)", (double)ratio);
    }
    
    int quadrant_counts[4] = {0, 0, 0, 0};
    for (int i = 0; i < stored_samples; i++) {
        float x_centered = samples_x[i] - g_mag_offset_x;
        float y_centered = samples_y[i] - g_mag_offset_y;
        
        if (x_centered >= 0 && y_centered >= 0) quadrant_counts[0]++;
        else if (x_centered < 0 && y_centered >= 0) quadrant_counts[1]++;
        else if (x_centered < 0 && y_centered < 0) quadrant_counts[2]++;
        else quadrant_counts[3]++;
    }
    
    // Check if all quadrants were covered reasonably
    int min_quadrant = quadrant_counts[0];
    int max_quadrant = quadrant_counts[0];
    for (int i = 1; i < 4; i++) {
        if (quadrant_counts[i] < min_quadrant) min_quadrant = quadrant_counts[i];
        if (quadrant_counts[i] > max_quadrant) max_quadrant = quadrant_counts[i];
    }
    
    if (min_quadrant < stored_samples / 8) {
        LOG_WRN("Robot may not have completed full rotation");
    }
    
    float expected_magnitude = avg_range;

    settings_save_float("calibration/mag_offset_x", g_mag_offset_x);
    settings_save_float("calibration/mag_offset_y", g_mag_offset_y);
    settings_save_float("calibration/mag_scale_x", g_mag_scale_x);
    settings_save_float("calibration/mag_scale_y", g_mag_scale_y);
}

void cal_motor_drive_asymmetry(void) {
    k_sleep(K_MSEC(3000));
    
    float fwd_left[3], fwd_right[3];
    
    for (int test = 0; test < 3; test++) {
        k_sleep(K_MSEC(2000));
        
        int32_t start_left, start_right;
        mb_angle(&start_left, &start_right);
        
        mb_drive(SPEED, SPEED);
        k_sleep(K_MSEC(2000));
        mb_drive(0, 0);
        
        int32_t end_left, end_right;
        mb_angle(&end_left, &end_right);
        
        fwd_left[test] = abs(end_left - start_left);
        fwd_right[test] = abs(end_right - start_right);
        
        k_sleep(K_MSEC(2000));
    }
    
    float bwd_left[3], bwd_right[3];
    
    for (int test = 0; test < 3; test++) {
        k_sleep(K_MSEC(2000));
        
        int32_t start_left, start_right;
        mb_angle(&start_left, &start_right);
        
        mb_drive(-SPEED, -SPEED);
        k_sleep(K_MSEC(2000));
        mb_drive(0, 0);
        
        int32_t end_left, end_right;
        mb_angle(&end_left, &end_right);
        
        bwd_left[test] = abs(end_left - start_left);
        bwd_right[test] = abs(end_right - start_right);
        
        k_sleep(K_MSEC(2000));
    }
    
    float avg_fwd_left = (fwd_left[0] + fwd_left[1] + fwd_left[2]) / 3.0f;
    float avg_fwd_right = (fwd_right[0] + fwd_right[1] + fwd_right[2]) / 3.0f;
    float avg_bwd_left = (bwd_left[0] + bwd_left[1] + bwd_left[2]) / 3.0f;
    float avg_bwd_right = (bwd_right[0] + bwd_right[1] + bwd_right[2]) / 3.0f;
    
    float left_ratio = (avg_fwd_left + avg_bwd_left) / 2.0f;
    float right_ratio = (avg_fwd_right + avg_bwd_right) / 2.0f;
    
    float target = (left_ratio + right_ratio) / 2.0f;
    g_motor_left_scale = target / left_ratio;
    g_motor_right_scale = target / right_ratio;
    
    if (g_motor_left_scale > 2.0f) g_motor_left_scale = 2.0f;
    if (g_motor_left_scale < 0.5f) g_motor_left_scale = 0.5f;
    if (g_motor_right_scale > 2.0f) g_motor_right_scale = 2.0f;
    if (g_motor_right_scale < 0.5f) g_motor_right_scale = 0.5f;
    
    settings_save_float("calibration/motor_left_scale", g_motor_left_scale);
    settings_save_float("calibration/motor_right_scale", g_motor_right_scale);
}

static void cal_turn_asymmetry(void) {
    k_sleep(K_MSEC(2000));
    
    lsm6dsox_gyro_data_t gyro;

    float cw_rates[3];
    
    for (int i = 0; i < 3; i++) {
        mb_drive(-TURNSPEED, TURNSPEED);
        k_sleep(K_MSEC(500));  // Stabilize
        
        float sum = 0.0f;
        int samples = 0;
        for (int j = 0; j < 50; j++) {
            if (lsm6dsox_read_gyro(&gyro) == 0) {
                float dps = fabsf(lsm6dsox_gyro_to_dps(gyro.z) - g_gyro_bias_z);
                sum += dps;
                samples++;
            }
            k_sleep(K_MSEC(10));
        }
        mb_drive(0, 0);
        
        cw_rates[i] = sum / samples;
        LOG_INF("  CW test %d: %.2f deg/s", i + 1, (double)cw_rates[i]);
        k_sleep(K_MSEC(1500));
    }
    
    float ccw_rates[3];
    
    for (int i = 0; i < 3; i++) {
        mb_drive(TURNSPEED, -TURNSPEED);
        k_sleep(K_MSEC(500));  // Stabilize
        
        float sum = 0.0f;
        int samples = 0;
        for (int j = 0; j < 50; j++) {
            if (lsm6dsox_read_gyro(&gyro) == 0) {
                float dps = fabsf(lsm6dsox_gyro_to_dps(gyro.z) - g_gyro_bias_z);
                sum += dps;
                samples++;
            }
            k_sleep(K_MSEC(10));
        }
        mb_drive(0, 0);
        
        ccw_rates[i] = sum / samples;
        LOG_INF("  CCW test %d: %.2f deg/s", i + 1, (double)ccw_rates[i]);
        k_sleep(K_MSEC(1500));
    }
    
    float avg_cw = (cw_rates[0] + cw_rates[1] + cw_rates[2]) / 3.0f;
    float avg_ccw = (ccw_rates[0] + ccw_rates[1] + ccw_rates[2]) / 3.0f;
    
    // Scale to match the slower direction
    float target_rate = (avg_cw < avg_ccw) ? avg_cw : avg_ccw;
    g_turn_cw_scale = target_rate / avg_cw;
    g_turn_ccw_scale = target_rate / avg_ccw;
    
    settings_save_float("calibration/turn_cw_scale", g_turn_cw_scale);
    settings_save_float("calibration/turn_ccw_scale", g_turn_ccw_scale);
}

static void cal_stopping_asymmetry(void) {
    LOG_INF("\n=== Stopping Asymmetry Calibration ===");
    LOG_INF("Robot will test stopping behavior");
    k_sleep(K_MSEC(2000));
    
    lsm6dsox_gyro_data_t gyro;
    
    // Test CW stopping
    LOG_INF("\nTesting CW stops (3 samples)...");
    int64_t cw_stop_times[3];
    
    for (int i = 0; i < 3; i++) {
        mb_drive(-TURNSPEED, TURNSPEED);
        k_sleep(K_MSEC(1000));
        
        int64_t stop_start = k_uptime_get();
        mb_drive(0, 0);
        
        float current_rate = 100.0f;
        while (current_rate > 5.0f && (k_uptime_get() - stop_start) < 2000) {
            if (lsm6dsox_read_gyro(&gyro) == 0) {
                current_rate = fabsf(lsm6dsox_gyro_to_dps(gyro.z) - g_gyro_bias_z);
            }
            k_sleep(K_MSEC(5));
        }
        
        cw_stop_times[i] = k_uptime_get() - stop_start;
        LOG_INF("  CW stop %d: %lld ms", i + 1, cw_stop_times[i]);
        k_sleep(K_MSEC(2000));
    }
    
    // Test CCW stopping
    LOG_INF("\nTesting CCW stops (3 samples)...");
    int64_t ccw_stop_times[3];
    
    for (int i = 0; i < 3; i++) {
        mb_drive(TURNSPEED, -TURNSPEED);
        k_sleep(K_MSEC(1000));
        
        int64_t stop_start = k_uptime_get();
        mb_drive(0, 0);
        
        float current_rate = 100.0f;
        while (current_rate > 5.0f && (k_uptime_get() - stop_start) < 2000) {
            if (lsm6dsox_read_gyro(&gyro) == 0) {
                current_rate = fabsf(lsm6dsox_gyro_to_dps(gyro.z) - g_gyro_bias_z);
            }
            k_sleep(K_MSEC(5));
        }
        
        ccw_stop_times[i] = k_uptime_get() - stop_start;
        LOG_INF("  CCW stop %d: %lld ms", i + 1, ccw_stop_times[i]);
        k_sleep(K_MSEC(2000));
    }
    
    int64_t avg_cw = (cw_stop_times[0] + cw_stop_times[1] + cw_stop_times[2]) / 3;
    int64_t avg_ccw = (ccw_stop_times[0] + ccw_stop_times[1] + ccw_stop_times[2]) / 3;
    
    if (avg_cw > avg_ccw) {
        float ratio = (float)avg_cw / (float)avg_ccw;
        ratio = 1.0f + (ratio - 1.0f) * 1.2f;
        g_cw_stop_ahead = 0.7f * ratio;
        g_ccw_stop_ahead = 0.7f;
        g_cw_brake_scale = 1.0f * ratio;
        g_ccw_brake_scale = 1.0f;
    } else {
        float ratio = (float)avg_ccw / (float)avg_cw;
        ratio = 1.0f + (ratio - 1.0f) * 1.2f;
        g_cw_stop_ahead = 0.7f;
        g_ccw_stop_ahead = 0.7f * ratio;
        g_cw_brake_scale = 1.0f;
        g_ccw_brake_scale = 1.0f * ratio;
    }
    
    if (g_cw_stop_ahead > 2.5f) g_cw_stop_ahead = 2.5f;
    if (g_ccw_stop_ahead > 2.5f) g_ccw_stop_ahead = 2.5f;
    if (g_cw_brake_scale > 1.8f) g_cw_brake_scale = 1.8f;
    if (g_ccw_brake_scale > 1.8f) g_ccw_brake_scale = 1.8f;
    
    settings_save_float("calibration/cw_stop_ahead", g_cw_stop_ahead);
    settings_save_float("calibration/ccw_stop_ahead", g_ccw_stop_ahead);
    settings_save_float("calibration/cw_brake_scale", g_cw_brake_scale);
    settings_save_float("calibration/ccw_brake_scale", g_ccw_brake_scale);
}

void calibration_sequence(int16_t spd, void (*drv)(int16_t, int16_t), void (*led)(int)) {
    ARG_UNUSED(spd);
    
    if (led) {
        for (int i = 0; i < 3; i++) {
            led(1);
            k_sleep(K_MSEC(200));
            led(0);
            k_sleep(K_MSEC(200));
        }
    }
    
    k_sleep(K_MSEC(2000));
    
    // IMU
    cal_accel();
    k_sleep(K_MSEC(500));
    
    cal_gyro();
    k_sleep(K_MSEC(1000));
    
    k_sleep(K_MSEC(1000));
    mag_seq(40000, drv, led);
    
    k_sleep(K_MSEC(2000));
    
    // Encoder
    k_sleep(K_MSEC(2000));
    cal_encoders(3000, drv, led);
    
    k_sleep(K_MSEC(2000));
    
    // Motor asymmetry
    k_sleep(K_MSEC(2000));
    
    cal_motor_drive_asymmetry();
    k_sleep(K_MSEC(2000));
    
    cal_turn_asymmetry();
    k_sleep(K_MSEC(2000));
    
    cal_stopping_asymmetry();
}
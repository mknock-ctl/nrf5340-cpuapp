#include "robot/calibration.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/settings.h"
#include "ses_assignment.h"
#include <errno.h>
#include <stdio.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(calibration, LOG_LEVEL_DBG);

float g_gyro_bias_z = 0.0f;
float g_ms_per_degree = 0.0f;

float g_mag_offset_x = -1.0f;
float g_mag_offset_y = -1.0f;
float g_mag_scale_x = -1.0f;
float g_mag_scale_y = -1.0f;
float g_drive_avg_duration_ms = 0.0f;

static const setting_entry_t robot_settings[] = {
    {"bias_z", &g_gyro_bias_z, sizeof(g_gyro_bias_z)},
    {"ms_per_deg", &g_ms_per_degree, sizeof(g_ms_per_degree)},
    {"mag_offset_x", &g_mag_offset_x, sizeof(g_mag_offset_x)},
    {"mag_offset_y", &g_mag_offset_y, sizeof(g_mag_offset_y)},
    {"mag_scale_x", &g_mag_scale_x, sizeof(g_mag_scale_x)},
    {"mag_scale_y", &g_mag_scale_y, sizeof(g_mag_scale_y)},
    {"drive_avg_duration_ms", &g_drive_avg_duration_ms, sizeof(g_drive_avg_duration_ms)}};

int calibration_init(bool reset) {
    TRY_ERR(int,
            settings_init_and_register("calibration", robot_settings, ARRAY_SIZE(robot_settings)));

    if (reset) {
        const float g_reset = 0.0f;
        settings_save_float("calibration/ms_per_deg", g_reset);
        g_ms_per_degree = 0.0f;

        settings_save_float("calibration/drive_avg_duration_ms", g_drive_avg_duration_ms);
        g_drive_avg_duration_ms = 0.0f;

        const float mag_reset = -1.0f;
        settings_save_float("calibration/mag_offset_x", mag_reset);
        settings_save_float("calibration/mag_offset_y", mag_reset);
        g_mag_offset_x = -1.0f;
        g_mag_offset_y = -1.0f;
        /* The scales change with the offsets */
    }

    LOG_INF("Booted. Bias: %.3f, MS/Deg: %.3f, magx: %.3f, maxy: %.3f,", (double)g_gyro_bias_z,
            (double)g_ms_per_degree, (double)g_mag_offset_x, (double)g_mag_offset_y);
    LOG_INF("Booted 2. MagScaleX: %.3f, MagScaleY: %.3f, DriveAvgDur: %.3f", (double)g_mag_scale_x,
            (double)g_mag_scale_y, (double)g_drive_avg_duration_ms);
    return 0;
}

bool calibration_needed(void) {
    return (fabsf(g_ms_per_degree) < 0.001f || fabsf(g_drive_avg_duration_ms) < 0.001f ||
            g_mag_offset_x < 0 || g_mag_offset_y < 0);
}

static void calibrate_gyro_offset(void) {
    LOG_INF("Calibrating gyro bias (keep robot still!)...");
    lsm6dsox_gyro_data_t gyro;
    float sum_z = 0.0f;
    int samples = 100;

    for (int i = 0; i < samples; i++) {
        if (lsm6dsox_read_gyro(&gyro) == 0) {
            sum_z += lsm6dsox_gyro_to_dps(gyro.z);
        }
        k_sleep(K_MSEC(10));
    }

    g_gyro_bias_z = sum_z / samples;
    settings_save_float("calibration/bias_z", g_gyro_bias_z);
    LOG_INF("Gyro Bias calibrated: %.2f dps", (double)g_gyro_bias_z);
}

static float calibrate_turn_rate_internal(int16_t speed, uint32_t duration_ms,
                                          void (*drive_func)(int16_t, int16_t)) {
    lsm6dsox_gyro_data_t gyro;
    float total_rotation = 0.0f;

    LOG_INF("Starting turn measurement: speed=%d", speed);

    drive_func(speed, -speed);

    int64_t start_time = k_uptime_get();
    int64_t last_sample_time = start_time;

    while ((k_uptime_get() - start_time) < duration_ms) {
        if (lsm6dsox_read_gyro(&gyro) == 0) {
            int64_t current_time = k_uptime_get();
            float dt = (current_time - last_sample_time) / 1000.0f; // Seconds

            float gyro_z_dps = lsm6dsox_gyro_to_dps(gyro.z) - g_gyro_bias_z;

            // Integrate
            total_rotation += fabsf(gyro_z_dps * dt);

            last_sample_time = current_time;
        }
        k_sleep(K_MSEC(10));
    }

    drive_func(0, 0);

    float actual_duration = (k_uptime_get() - start_time) / 1000.0f;
    float avg_dps = total_rotation / actual_duration;

    if (avg_dps < 0.1f)
        avg_dps = 0.1f;

    float ms_per_degree = 1000.0f / avg_dps;
    return ms_per_degree;
}

static float gyro_calibration_sequence(int16_t speed, uint32_t duration_ms,
                                       void (*drive_func)(int16_t, int16_t),
                                       void (*led_func)(int)) {
    calibrate_gyro_offset();
    const int num_runs = 3;
    float sum_ms_per_deg = 0.0f;
    LOG_INF("Starting Sequence: Runs=%d, Speed=%d", num_runs, speed);

    for (int i = 0; i < num_runs; i++) {
        LOG_INF("--- Run %d/%d ---", i + 1, num_runs);

        if (led_func) {
            led_func(i);
        }

        k_sleep(K_MSEC(1000));

        float ms_per_deg = calibrate_turn_rate_internal(speed, duration_ms, drive_func);
        sum_ms_per_deg += ms_per_deg;

        k_sleep(K_MSEC(1000));
    }

    float avg_ms_per_deg = sum_ms_per_deg / num_runs;

    LOG_INF("Result: %.3f ms/deg (%.2f deg/s)", (double)avg_ms_per_deg,
            (double)(1000.0f / avg_ms_per_deg));

    g_ms_per_degree = avg_ms_per_deg;
    settings_save_float("calibration/ms_per_deg", g_ms_per_degree);
    return avg_ms_per_deg;
}

static void calibrate_magnetometer_sequence(uint32_t duration_ms,
                                            void (*drive_func)(int16_t, int16_t),
                                            void (*led_func)(int)) {
    LOG_INF("Robot will spin for %.3f seconds. Keep it on the floor!",
            (double)(duration_ms / 1000));

    int16_t min_x = 32000, max_x = -32000;
    int16_t min_y = 32000, max_y = -32000;
    lis3mdl_data_t data;
    int sample_count = 0;

    drive_func(TURNSPEED, -TURNSPEED);
    uint32_t start_time = k_uptime_get_32();

    if (led_func) {
        led_func(1);
    }

    float x_filtered = 0.0f, y_filtered = 0.0f;
    const float alpha = 0.1f;
    bool initialized = false;

    while (k_uptime_get_32() - start_time < duration_ms) {
        TRY_ERR(int, lis3mdl_read_mag(&data));

        /* low-pass filter (smooth noise) */
        if (!initialized) {
            x_filtered = (float)data.x;
            y_filtered = (float)data.y;
            initialized = true;
        } else {
            x_filtered = alpha * (float)data.x + (1.0f - alpha) * x_filtered;
            y_filtered = alpha * (float)data.y + (1.0f - alpha) * y_filtered;
        }

        int32_t x_val = (int32_t)roundf(x_filtered);
        int32_t y_val = (int32_t)roundf(y_filtered);

        if (x_val < min_x)
            min_x = x_val;
        if (x_val > max_x)
            max_x = x_val;
        if (y_val < min_y)
            min_y = y_val;
        if (y_val > max_y)
            max_y = y_val;

        sample_count++;
        k_sleep(K_MSEC(20));
    }

    drive_func(0, 0);
    if (led_func) {
        led_func(1);
    }

    g_mag_offset_x = ((float)min_x + (float)max_x) * 0.5f;
    g_mag_offset_y = ((float)min_y + (float)max_y) * 0.5f;

    /* Because X is way too high => give axes same half-range */
    float range_x = (float)(max_x - min_x) * 0.5f;
    float range_y = (float)(max_y - min_y) * 0.5f;
    float avg_range = (range_x + range_y) * 0.5f;

    if (range_x > 0.1f && range_y > 0.1f) {
        g_mag_scale_x = avg_range / range_x;
        g_mag_scale_y = avg_range / range_y;
    } else {
        g_mag_scale_x = 1.0f;
        g_mag_scale_y = 1.0f;
    }

    settings_save_float("calibration/mag_offset_x", g_mag_offset_x);
    settings_save_float("calibration/mag_offset_y", g_mag_offset_y);
    settings_save_float("calibration/mag_scale_x", g_mag_scale_x);
    settings_save_float("calibration/mag_scale_y", g_mag_scale_y);
}

static float calibrate_distance_internal(int16_t speed, uint32_t duration_ms,
                                         void (*drive_func)(int16_t, int16_t)) {
    drive_func(speed, speed);

    int64_t start_time = k_uptime_get();
    k_sleep(K_MSEC(duration_ms));

    drive_func(0, 0);

    int64_t actual_duration = k_uptime_get() - start_time;

    return (float)actual_duration;
}

static float distance_calibration_sequence(int16_t speed, uint32_t duration_ms,
                                           void (*drive_func)(int16_t, int16_t),
                                           void (*led_func)(int)) {
    const int num_runs = 3;
    float total_duration = 0.0f;

    for (int i = 0; i < num_runs; i++) {
        if (led_func) {
            led_func(i);
        }

        k_sleep(K_MSEC(2000));

        float duration = calibrate_distance_internal(speed, duration_ms, drive_func);
        total_duration += duration;

        k_sleep(K_MSEC(2000));
    }
    float avg_duration = total_duration / num_runs;

    LOG_INF("Example: if robot traveled 500mm, then mm_per_ms = 500 / %.1f = %.3f",
            (double)avg_duration, 500.0 / (double)avg_duration);

    settings_save_float("calibration/drive_avg_duration_ms", avg_duration);
    return avg_duration;
}

void calibration_sequence(int16_t speed, void (*drive_func)(int16_t, int16_t),
                          void (*led_func)(int)) {
    gyro_calibration_sequence(speed, 2000, drive_func, led_func);
    calibrate_magnetometer_sequence(10000, drive_func, led_func);
    distance_calibration_sequence(speed, 2000, drive_func, led_func);
}

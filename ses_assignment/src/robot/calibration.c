#include "robot/calibration.h"
#include "robot/sensors/lsm6dsox.h"
#include "ses_assignment.h"
#include <errno.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(calibration, LOG_LEVEL_DBG);

float g_gyro_bias_z = 0.0f;
float g_ms_per_degree = 0.0f;

static int robot_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg);

static struct settings_handler robot_conf = {.name = "robot", .h_set = robot_settings_set};

static int robot_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                              void *cb_arg) {
    const char *next;
    int rc;

    if (settings_name_steq(name, "bias_z", &next) && !next) {
        if (len != sizeof(g_gyro_bias_z))
            return -EINVAL;
        rc = read_cb(cb_arg, &g_gyro_bias_z, sizeof(g_gyro_bias_z));
        LOG_INF("Restored Gyro Bias: %.3f", (double)g_gyro_bias_z);
        return 0;
    }

    if (settings_name_steq(name, "ms_per_deg", &next) && !next) {
        if (len != sizeof(g_ms_per_degree))
            return -EINVAL;
        rc = read_cb(cb_arg, &g_ms_per_degree, sizeof(g_ms_per_degree));
        LOG_INF("Restored MS Per Deg: %.3f", (double)g_ms_per_degree);
        return 0;
    }

    return -ENOENT;
}

int calibration_init(bool reset) {
    TRY_ERR(int, settings_subsys_init());

    if (reset) {
        const float g_reset = 0.0f;
        settings_save_one("robot/ms_per_deg", &g_reset, sizeof(g_reset));
        g_ms_per_degree = 0.0f;
    }

    settings_register(&robot_conf);
    settings_load();
    LOG_INF("Booted. Bias: %.3f, MS/Deg: %.3f", (double)g_gyro_bias_z, (double)g_ms_per_degree);
    return 0;
}

bool calibration_needed(void) { return (fabsf(g_ms_per_degree) < 0.001f); }

void calibrate_gyro_offset(void) {
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
    settings_save_one("robot/bias_z", &g_gyro_bias_z, sizeof(g_gyro_bias_z));
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

float calibration_sequence(int16_t speed, void (*drive_func)(int16_t, int16_t),
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

        float ms_per_deg = calibrate_turn_rate_internal(speed, 2000, drive_func);
        sum_ms_per_deg += ms_per_deg;

        k_sleep(K_MSEC(1000));
    }

    float avg_ms_per_deg = sum_ms_per_deg / num_runs;

    LOG_INF("Result: %.3f ms/deg (%.2f deg/s)", (double)avg_ms_per_deg,
            (double)(1000.0f / avg_ms_per_deg));

    g_ms_per_degree = avg_ms_per_deg;
    settings_save_one("robot/ms_per_deg", &g_ms_per_degree, sizeof(g_ms_per_degree));

    return avg_ms_per_deg;
}

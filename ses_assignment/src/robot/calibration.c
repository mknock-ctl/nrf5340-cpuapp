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

float g_accel_bias_x = 0.0f;

static const setting_entry_t robot_settings[] = {
    {"bias_z", &g_gyro_bias_z, sizeof(g_gyro_bias_z)},
    {"ms_per_deg", &g_ms_per_degree, sizeof(g_ms_per_degree)},
    {"mag_offset_x", &g_mag_offset_x, sizeof(g_mag_offset_x)},
    {"mag_offset_y", &g_mag_offset_y, sizeof(g_mag_offset_y)},
    {"mag_scale_x", &g_mag_scale_x, sizeof(g_mag_scale_x)},
    {"mag_scale_y", &g_mag_scale_y, sizeof(g_mag_scale_y)},
    {"drive_avg_duration_ms", &g_drive_avg_duration_ms, sizeof(g_drive_avg_duration_ms)},
    {"accel_bias_x", &g_accel_bias_x, sizeof(g_accel_bias_x)}
};

int calibration_init(bool reset) {
    TRY_ERR(int, settings_init_and_register("calibration", robot_settings, ARRAY_SIZE(robot_settings)));

    if (reset) {
        const float zero = 0.0f;
        const float neg = -1.0f;
        
        settings_save_float("calibration/ms_per_deg", zero);
        settings_save_float("calibration/drive_avg_duration_ms", zero);
        settings_save_float("calibration/mag_offset_x", neg);
        settings_save_float("calibration/mag_offset_y", neg);
        settings_save_float("calibration/accel_bias_x", zero);
        
        g_ms_per_degree = 0.0f;
        g_drive_avg_duration_ms = 0.0f;
        g_mag_offset_x = -1.0f;
        g_mag_offset_y = -1.0f;
        g_accel_bias_x = 0.0f;
    }

    LOG_INF("Booted. Bias: %.3f, MS/Deg: %.3f, magx: %.3f, maxy: %.3f,", (double)g_gyro_bias_z,
            (double)g_ms_per_degree, (double)g_mag_offset_x, (double)g_mag_offset_y);
    LOG_INF("Booted 2. MagScaleX: %.3f, MagScaleY: %.3f, DriveAvgDur: %.3f", (double)g_mag_scale_x,
            (double)g_mag_scale_y, (double)g_drive_avg_duration_ms);
    LOG_INF("Booted 3. AccelBiasX: %.3f", (double)g_accel_bias_x);
    return 0;
}

bool calibration_needed(void) {
    return (fabsf(g_ms_per_degree) < 0.001f || 
            fabsf(g_drive_avg_duration_ms) < 0.001f ||
            g_mag_offset_x < 0 || g_mag_offset_y < 0 ||
            fabsf(g_accel_bias_x) < 0.001f);
}

static void cal_gyro(void) {
    lsm6dsox_gyro_data_t g;
    float sum = 0.0f;

    for (int i = 0; i < 100; i++) {
        if (lsm6dsox_read_gyro(&g) == 0) {
            sum += lsm6dsox_gyro_to_dps(g.z);
        }
        k_sleep(K_MSEC(10));
    }

    g_gyro_bias_z = sum / 100.0f;
    settings_save_float("calibration/bias_z", g_gyro_bias_z);
}

static void cal_accel(void) {
    int16_t x, y, z;
    int32_t sum = 0;

    for (int i = 0; i < 100; i++) {
        if (lsm6dsox_read_accel_raw(&x, &y, &z) == 0) {
            sum += x;
        }
        k_sleep(K_MSEC(10));
    }

    g_accel_bias_x = sum / 100.0f;
    settings_save_float("calibration/accel_bias_x", g_accel_bias_x);
}

static float cal_turn_rate(int16_t spd, uint32_t dur, void (*drv)(int16_t, int16_t)) {
    lsm6dsox_gyro_data_t g;
    float rot = 0.0f;

    drv(spd, -spd);

    int64_t t0 = k_uptime_get();
    int64_t t_last = t0;

    while ((k_uptime_get() - t0) < dur) {
        if (lsm6dsox_read_gyro(&g) == 0) {
            int64_t t_now = k_uptime_get();
            float dt = (t_now - t_last) / 1000.0f;
            // Integrate angular velocity
            rot += fabsf(lsm6dsox_gyro_to_dps(g.z) - g_gyro_bias_z) * dt;
            t_last = t_now;
        }
        k_sleep(K_MSEC(10));
    }

    drv(0, 0);

    float dur_s = (k_uptime_get() - t0) / 1000.0f;
    float dps = rot / dur_s;
    return (dps < 0.1f) ? 10000.0f : (1000.0f / dps);
}

static float gyro_seq(int16_t spd, uint32_t dur, void (*drv)(int16_t, int16_t), void (*led)(int)) {
    cal_gyro();
    
    float sum = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (led) led(i);
        k_sleep(K_MSEC(1000));
        sum += cal_turn_rate(spd, dur, drv);
        k_sleep(K_MSEC(1000));
    }

    g_ms_per_degree = sum / 3.0f;
    settings_save_float("calibration/ms_per_deg", g_ms_per_degree);
    return g_ms_per_degree;
}

static void mag_seq(uint32_t dur, void (*drv)(int16_t, int16_t), void (*led)(int)) {
    int16_t min_x = 32000, max_x = -32000;
    int16_t min_y = 32000, max_y = -32000;
    lis3mdl_data_t d;

    drv(MIN_SPEED, -MIN_SPEED);
    uint32_t t0 = k_uptime_get_32();

    if (led) led(1);

    float xf = 0, yf = 0;
    bool init = false;
    const float a = 0.9f;

    while (k_uptime_get_32() - t0 < dur) {
        if (lis3mdl_read_mag(&d) != 0) continue;

        if (!init) {
            xf = d.x;
            yf = d.y;
            init = true;
        } else {
            xf = a * d.x + (1.0f - a) * xf;
            yf = a * d.y + (1.0f - a) * yf;
        }

        int16_t x = (int16_t)roundf(xf);
        int16_t y = (int16_t)roundf(yf);

        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;

        k_sleep(K_MSEC(20));
    }

    drv(0, 0);
    if (led) led(1);

    g_mag_offset_x = (min_x + max_x) * 0.5f;
    g_mag_offset_y = (min_y + max_y) * 0.5f;

    float rx = (max_x - min_x) * 0.5f;
    float ry = (max_y - min_y) * 0.5f;
    float avg = (rx + ry) * 0.5f;

    if (rx > 0.1f && ry > 0.1f) {
        g_mag_scale_x = avg / rx;
        g_mag_scale_y = avg / ry;
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

void calibration_sequence(int16_t spd, void (*drv)(int16_t, int16_t), void (*led)(int)) {
    cal_accel();
    gyro_seq(spd, 2000, drv, led);
    mag_seq(30000, drv, led);
    //distance_calibration_sequence(speed, 2000, drive_func, led_func);
}

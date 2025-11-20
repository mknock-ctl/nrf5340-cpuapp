#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include "ses_assignment.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <mergebot.h>

#include "lis3mdl.h"
#include "lsm6dsox.h"
#include "power.h"
#include "robot.h"
#include "tap_detect.h"
#include "mb_power.h"
#include <math.h>

#define DELAY 500
#define TAP_TIMEOUT_MS 100

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int init_hardware(void)
{
    LOG_INF("Initializing hardware...");

    TRY_ERR(mb_error_t, mb_measure_init(0));
    TRY_ERR(mb_error_t, mb_drive_init(4));
    TRY_ERR(mb_error_t, mb_power_init());
    TRY_ERR(mb_error_t, mb_led_init());

    LOG_INF("Hardware ready");
    return 0;
}

float calibrate_turn_rate(int16_t speed, uint32_t duration_ms) {
    lsm6dsox_gyro_data_t gyro;
    float total_rotation = 0.0f;
    int sample_count = 0;
    
    LOG_INF("Starting calibration: speed=%d, duration=%dms", speed, duration_ms);
    
    // Start turning
    mb_drive(speed, -speed);
    
    int64_t start_time = k_uptime_get();
    int64_t last_sample_time = start_time;
    
    // Sample gyroscope while turning
    while ((k_uptime_get() - start_time) < duration_ms) {
        TRY_ERR(int, lsm6dsox_read_gyro(&gyro));

        int64_t current_time = k_uptime_get();
        float dt = (current_time - last_sample_time) / 1000.0f;  // to sec        

        float gyro_z_dps = lsm6dsox_gyro_to_dps(gyro.z);
        
        total_rotation += gyro_z_dps * dt;
        
        sample_count++;
        last_sample_time = current_time;
        
        if (sample_count % 10 == 0) {
            LOG_DBG("Gyro X: %d raw, Gyro Y: %d raw, Gyro Z: %d raw, %.2f dps, total: %.2f deg", 
                    gyro.x, gyro.y, gyro.z, (double)gyro_z_dps, (double)total_rotation);
        }
        k_sleep(K_MSEC(10));  // 100Hz
    }

    mb_drive(0, 0);
    
    float actual_duration = (k_uptime_get() - start_time) / 1000.0f;
    float avg_dps = total_rotation / actual_duration;
    float ms_per_degree = 1000.0f / fabsf(avg_dps);
    
    LOG_INF("Calibration complete:");
    LOG_INF("  Duration: %.3f s", (double)actual_duration);
    LOG_INF("  Total rotation: %.2f degrees", (double)total_rotation);
    LOG_INF("  Average rate: %.2f deg/s", (double)avg_dps);
    LOG_INF("  MS per degree: %.3f", (double)ms_per_degree);
    LOG_INF("  Samples: %d", sample_count);
    
    return ms_per_degree;
}

float run_calibration_sequence(int16_t speed) {
    const int num_runs = 3;
    float sum_ms_per_deg = 0.0f;
    
    LOG_INF("=== Starting calibration sequence ===");
    LOG_INF("Number of runs: %d", num_runs);
    LOG_INF("Motor speed: %d", speed);
    
    for (int i = 0; i < num_runs; i++) {
        LOG_INF("--- Run %d/%d ---", i + 1, num_runs);
        
        for (int j = 0; j <= i; j++) {
            mb_led_toggle(MB_LED_G);
            k_sleep(K_MSEC(200));
            mb_led_toggle(MB_LED_G);
            k_sleep(K_MSEC(200));
        }
        
        k_sleep(K_MSEC(500));
        
        // Turn for 360 degrees (approximately)
        float ms_per_deg = calibrate_turn_rate(speed, 3000);
        sum_ms_per_deg += ms_per_deg;
        
        k_sleep(K_MSEC(1000));
    }
    
    float avg_ms_per_deg = sum_ms_per_deg / num_runs;
    
    LOG_INF("========== cal res ========");
    LOG_INF("Average MS per degree: %.3f", (double) avg_ms_per_deg);
    LOG_INF("Expected time for 360°: %.0f ms", (double) (avg_ms_per_deg * 360.0f));
    LOG_INF("Expected turn rate: %.2f deg/s", (double) (1000.0f / avg_ms_per_deg));
    
    for (int i = 0; i < 6; i++) {
        mb_led_toggle(MB_LED_G);
        k_sleep(K_MSEC(100));
    }
    
    return avg_ms_per_deg;
}

/// @brief Showcase the robot capabilities
/// @todo Remove this function for the assignment
void showcase(void) {
    // Measure voltages
    int vcap = mb_measure_vcap();
    int vin = mb_measure_vin();
    LOG_INF("Vcap: %4dmV, Vin: %4dmV", vcap, vin);

    // Flash LEDs
    mb_led_toggle(MB_LED_R);
    mb_led_toggle(MB_LED_G);
    mb_led_toggle(MB_LED_B);

    // Drive motors
    mb_drive(-SPEED, SPEED);
    k_sleep(K_MSEC(DELAY));
    mb_drive(0, 0);
    k_sleep(K_MSEC(DELAY));
    mb_drive(SPEED, -SPEED);
    k_sleep(K_MSEC(DELAY));
    mb_drive(0, 0);
    k_sleep(K_MSEC(DELAY));
}

static void wait_for_double_tap(void) {
    LOG_INF("waiting for double tap");

    while (!tap_detect_wait(K_MSEC(TAP_TIMEOUT_MS))) {
        power_update_indicator();
    }

    mb_leds_off();
    LOG_INF("double tap detected");
}

int main(void) {
    __ASSERT(init_hardware() == 0, "Failed to initialize robot");

    mb_out_bumper_off();
    mb_leds_off();

    TRY_ERR(int, tap_detect_init());
    TRY_ERR(int, lis3mdl_init());

    LOG_INF("Robot initialization succeeded");

    for (;;) {
        int vcap = mb_measure_vcap();
        int vin = mb_measure_vin();
        LOG_INF("Vcap: %4dmV, Vin: %4dmV", vcap, vin);       
        
        float ms_per_degree = run_calibration_sequence(TURNSPEED);

        wait_for_double_tap();
        //robot_turn_to_north();
        //robot_turn(TURNSPEED);
        int32_t turn_time_ms = (int32_t)(fabsf((360.0f) * ms_per_degree));
        
        TRY_ERR(mb_error_t, mb_drive(TURNSPEED, -TURNSPEED));  
        k_sleep(K_MSEC(turn_time_ms));        
        mb_drive(0, 0);
    }

    UNREACHABLE();
    return -1;
}

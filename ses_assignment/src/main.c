#include <math.h>
#include <stdbool.h>
#include <mergebot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led.h>


#include "mb_power.h"
#include "robot.h"
#include "robot/calibration.h"
#include "robot/power.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/sensors/tap_detect.h"
#include "robot/sensors/crash_detect.h"
#include "ses_assignment.h"

static const struct device *pwm_leds_dev = DEVICE_DT_GET(DT_NODELABEL(pwm_leds_controller));

#define LED_IDX_RED 0
#define LED_IDX_GRN 1

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
static K_SEM_DEFINE(tap_sem, 0, 1);

int init_hardware(void) {
    LOG_INF("Initializing hardware...");
    TRY_ERR(mb_error_t, mb_measure_init(0));
    TRY_ERR(mb_error_t, mb_drive_init(4));
    TRY_ERR(mb_error_t, mb_power_init());
    TRY_ERR(mb_error_t, mb_led_init());

    if (!device_is_ready(pwm_leds_dev)) {
        LOG_ERR("PWM LED controller not ready. Check DeviceTree overlay.");
        return -ENODEV;
    }

    LOG_INF("Hardware ready");
    return 0;
}

static void drive_callback(int16_t left, int16_t right) { mb_drive(left, right); }

static void led_indicator_callback(int index) {
    for (int j = 0; j <= index; j++) {
        mb_led_toggle(MB_LED_G);
        k_sleep(K_MSEC(200));
        mb_led_toggle(MB_LED_G);
        k_sleep(K_MSEC(200));
    }
}

static void update_battery_led(void) {
    uint8_t percentage = power_percentage();
    //LOG_INF("Battery level = %u%%", percentage);

    led_set_brightness(pwm_leds_dev, LED_IDX_GRN, percentage);
    led_set_brightness(pwm_leds_dev, LED_IDX_RED, 100 - percentage);
}

static void test_turn(void) {
    robot_turn(45);
    k_sleep(K_MSEC(400));
    robot_turn(-45);
    k_sleep(K_MSEC(400));
    robot_turn(90);
    k_sleep(K_MSEC(400));
    robot_turn(-90);
    k_sleep(K_MSEC(400));
    robot_turn(135);
    k_sleep(K_MSEC(400));
    robot_turn(-135);
    k_sleep(K_MSEC(400));
    robot_turn(180);
    k_sleep(K_MSEC(400));
    robot_turn(-180);
}

static void test_move(void) {
    robot_move(100);
    k_sleep(K_MSEC(3000));
    robot_move(-100);
    k_sleep(K_MSEC(3000));
    robot_move(500);
    k_sleep(K_MSEC(3000));
    robot_move(-500);
    k_sleep(K_MSEC(3000));
    robot_move(1000);
    k_sleep(K_MSEC(3000));
    robot_move(-1000);
    k_sleep(K_MSEC(3000));
    robot_move(2000);
    k_sleep(K_MSEC(3000));
    robot_move(-2000);
    k_sleep(K_MSEC(3000));
}

static void test_mag(void) {
    double x_raw_d, y_raw_d;
    lis3mdl_data_t mag_data;
    TRY_ERR(int, lis3mdl_read_mag(&mag_data));

    int32_t x_raw = mag_data.x;
    int32_t y_raw = mag_data.y;

    x_raw_d = (double)x_raw;
    y_raw_d = (double)y_raw;

    double x = x_raw_d;
    double y = y_raw_d;

    double x1 = x_raw_d - (double)g_mag_offset_x;
    double y1 = y_raw_d - (double)g_mag_offset_y;

    double x2 = (x_raw_d - (double)g_mag_offset_x) * (double)g_mag_scale_x;
    double y2 = (y_raw_d - (double)g_mag_offset_y) * (double)g_mag_scale_y;

    LOG_INF("(raw x:%.3f y:%.3f), (off x1:%.3f y1:%.3f), (corr x2:%.3f y2:%.3f)",
            x, y, x1, y1, x2, y2);

    double heading  = atan2(x, y) * 180.0 / M_PI;
    double heading1 = atan2(x1, y1) * 180.0 / M_PI;
    double heading2 = atan2(x2, y2) * 180.0 / M_PI;

    if (heading < 0)  heading += 360.0;
    if (heading1 < 0) heading1 += 360.0;
    if (heading2 < 0) heading2 += 360.0;

    LOG_INF("(h:%.3f), (h1:%.3f), (h2:%.3f), (h3: %.3f)", heading, heading1, heading2, robot_calculate_heading());

    k_sleep(K_MSEC(500));
}

static void tap_callback(void) {
    k_sem_give(&tap_sem);
}

static void wait_for_double_tap(void) {
    robot_set_imu_mode(IMU_MODE_TAP);
    tap_detect_register_callback(tap_callback);
    LOG_INF("Waiting for double tap...");
    
    k_sem_reset(&tap_sem);

    while (k_sem_take(&tap_sem, K_MSEC(100)) != 0) {
        update_battery_led();
    }    

    led_set_brightness(pwm_leds_dev, LED_IDX_RED, 0);
    led_set_brightness(pwm_leds_dev, LED_IDX_GRN, 0);

    // This reconfigures the pins back to GPIO Output mode so 
    // robot_set_status() works correctly.
    mb_led_init();

    robot_set_imu_mode(IMU_MODE_OFF);
    LOG_INF("Double tap detected.");
}

static void follow_predefined_path(float scale) {
    robot_move(2000 * scale);
    robot_turn(90); // 90 degrees left
    robot_move(2000 * scale);
    robot_turn(-90);
    robot_move(2000 * scale);
    robot_turn(-90);
    robot_move(2000 * scale);
    robot_turn(180);
    robot_move(-2000 * scale);
    robot_turn(90);
    robot_move(4000 * scale);        
}

static void first_assign(void) {
    robot_turn_to_north();
    robot_move(1000);
}

static void third_assign(void) {
    robot_move(10000);
}

int main(void) {
    __ASSERT(init_hardware() == 0, "Failed to initialize robot");
    mb_out_bumper_off();
    mb_leds_off();
    robot_init();

    if (calibration_needed()) {
        LOG_INF("Calibration required bro, double tap to start.");
        wait_for_double_tap();

        calibration_sequence(TURNSPEED, drive_callback, led_indicator_callback);
    }

    for (;;) {
        update_battery_led();

        wait_for_double_tap();

        k_sleep(K_MSEC(200));

        robot_set_imu_mode(IMU_MODE_CRASH);
        robot_set_status(STATUS_OK);

        robot_turn_to_north();
        k_sleep(K_MSEC(200));
        robot_move(500);

        wait_for_double_tap();
        k_sleep(K_MSEC(200));
        robot_set_imu_mode(IMU_MODE_CRASH);
        robot_set_status(STATUS_OK);
        for (int i = 0; i < 6; i++) {
            robot_move(400);
            k_sleep(K_MSEC(600));
            robot_turn(60);
            k_sleep(K_MSEC(600));
        }

        wait_for_double_tap();
        k_sleep(K_MSEC(200));

        robot_set_imu_mode(IMU_MODE_CRASH);
        robot_set_status(STATUS_OK);
        int32_t dist = 1000000;
        robot_move_with_factor(dist, (dist * 0.995));
    }

    UNREACHABLE();
    return 0;
}
#include <math.h>
#include <stdbool.h>
#include <mergebot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mb_power.h"
#include "robot.h"
#include "robot/calibration.h"
#include "robot/power.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot/sensors/tap_detect.h"
#include "ses_assignment.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int init_hardware(void) {
    LOG_INF("Initializing hardware...");
    TRY_ERR(mb_error_t, mb_measure_init(0));
    TRY_ERR(mb_error_t, mb_drive_init(4));
    TRY_ERR(mb_error_t, mb_power_init());
    TRY_ERR(mb_error_t, mb_led_init());
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

static void wait_for_double_tap(void) {
    LOG_INF("Waiting for double tap...");
    while (!tap_detect_wait(K_MSEC(TAP_TIMEOUT_MS))) {
        power_update_indicator();
    }
    LOG_INF("Double tap detected!");
}

int main(void) {
    __ASSERT(init_hardware() == 0, "Failed to initialize robot");
    mb_out_bumper_off();
    mb_leds_off();

    TRY_ERR(int, tap_detect_init());
    TRY_ERR(int, lis3mdl_init());
    TRY_ERR(int, calibration_init(CALIBRATION_RESET));

    if (calibration_needed()) {
        LOG_INF("Calibration required bro, double tap to start.");
        wait_for_double_tap();

        calibration_sequence(TURNSPEED, drive_callback, led_indicator_callback);
    }

    LOG_INF("Robot ready");
    wait_for_double_tap();

    for (int i = 0; i < 3; i++) {
        mb_led_toggle(MB_LED_G);
        k_sleep(K_MSEC(200));
    }

    for (;;) {
        int vcap = mb_measure_vcap();
        LOG_INF("Battery: %4dmV", vcap);

        robot_turn_to_north();

        wait_for_double_tap();
    }

    UNREACHABLE();
    return 0;
}

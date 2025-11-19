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

#define DELAY 500
#define TAP_TIMEOUT_MS 100

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int init_hardware(void)
{
    LOG_INF("Initializing hardware...");

    INIT_CHECK(mb_measure_init(0));
    INIT_CHECK(mb_drive_init(4));
    INIT_CHECK(mb_power_init());
    INIT_CHECK(mb_led_init());
    INIT_CHECK(tap_detect_init());
    INIT_CHECK(lis3mdl_init());

    LOG_INF("Hardware ready");
    return 0;
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

    LOG_INF("Robot initialization succeeded");

    for (;;) {
        int vcap = mb_measure_vcap();
        int vin = mb_measure_vin();
        LOG_INF("Vcap: %4dmV, Vin: %4dmV", vcap, vin);

        wait_for_double_tap();
        robot_turn_to_north();
    }

    UNREACHABLE();
    return -1;
}

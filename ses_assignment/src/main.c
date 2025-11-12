#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "robot.h"
#include "tap_detect.h"
#include "power.h"
#include <mergebot.h>

#define SPEED 100 // MAX 300
#define DELAY 500
#define TAP_TIMEOUT_MS 100

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/**
 * @brief Initialize the robot's hardware components
 * @retval `MB_SUCCESS` if all components initialized successfully
 * @retval Appropriate `mb_error_t` error code otherwise
 */
mb_error_t init(void) {
    // Initialize peripherals for vcap measurements
    TRY_ERR(mb_error_t, mb_measure_init(0));
    // Initialize peripherals for motor control
    TRY_ERR(mb_error_t, mb_drive_init(4));
    // Initialize peripherals for power management
    TRY_ERR(mb_error_t, mb_power_init());
    // Initialize peripherals for LED control
    TRY_ERR(mb_error_t, mb_led_init());

    // All initializations succeeded
    return MB_SUCCESS;
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

static void wait_for_double_tap(void)
{
    LOG_INF("waiting for double tap");

    while (!tap_detect_wait(K_MSEC(TAP_TIMEOUT_MS))) {
        power_update_indicator();
    }

    mb_leds_off();
    LOG_INF("double tap detected");
}

int main(void) {
    // Initialize robot
    __ASSERT(init() == MB_SUCCESS, "Failed to initialize robot");
    LOG_INF("Robot initialization succeeded");

    // Set initial state
    mb_out_bumper_off();
    mb_leds_off();

    tap_detect_init();

    // Main superloop
    for (;;) {
        //showcase();

        int vcap = mb_measure_vcap();
        int vin = mb_measure_vin();
        LOG_INF("Vcap: %4dmV, Vin: %4dmV", vcap, vin);

        // Functions to implement
        robot_move(200);
        robot_turn(90);
        robot_turn_to_north();
        wait_for_double_tap();
        robot_set_status(STATUS_CRASH);
    }

    // Unreachable
    UNREACHABLE();
    return -1;
}

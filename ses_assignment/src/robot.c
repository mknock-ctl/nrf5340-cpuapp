#include "robot.h"
#include "robot/calibration.h"
#include "robot/sensors/lis3mdl.h"
#include "robot/sensors/lsm6dsox.h"
#include "ses_assignment.h"
#include <math.h>
#include <mergebot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(robot, LOG_LEVEL_DBG);

static float normalize_angle_diff(float angle) {
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

float robot_calculate_heading(void) {
    lis3mdl_data_t mag_data;

    int err = lis3mdl_read_mag(&mag_data);
    if (err) {
        LOG_ERR("Mag read failed: %d", err);
        return 0.0f;
    }

    int16_t mag_x = mag_data.x - MAG_OFFSET_X;
    int16_t mag_y = mag_data.y - MAG_OFFSET_Y;

    // calc heading in radians
    float heading_rad = atan2f((float)mag_y, (float)mag_x);
    float heading_deg = RAD_TO_DEG(heading_rad);

    heading_deg = 90.0f - heading_deg;

    while (heading_deg >= 360.0f)
        heading_deg -= 360.0f;
    while (heading_deg < 0.0f)
        heading_deg += 360.0f;

    heading_deg += HEADING_OFFSET_DEG;
    if (heading_deg >= 360.0f)
        heading_deg -= 360.0f;
    if (heading_deg < 0.0f)
        heading_deg += 360.0f;

    LOG_DBG("Heading: %.1f (X:%d Y:%d)", (double)heading_deg, mag_data.x, mag_data.y);
    return heading_deg;
}

void robot_move(int32_t distance_mm) {
    LOG_INF("Move %d mm - Not Implemented", distance_mm);
    // TODO
}

void robot_turn(int32_t angle_deg) {
    if (angle_deg == 0)
        return;

    float target_angle = fabsf((float)angle_deg);
    float current_angle = 0.0f;
    lsm6dsox_gyro_data_t gyro;
    int64_t last_time = k_uptime_get();

    const float KP = 2.5f;                    // Proportional gain
    const int16_t MIN_SPEED = TURNSPEED / 10; // Minimum speed to prevent stalling
    int16_t base_speed = TURNSPEED;

    while (current_angle < target_angle) {
        if (lsm6dsox_read_gyro(&gyro) == 0) {
            int64_t now = k_uptime_get();
            if (now > last_time) {
                float dt = (now - last_time) / 1000.0f;
                last_time = now;

                float dps = lsm6dsox_gyro_to_dps(gyro.z) - g_gyro_bias_z;
                if (fabsf(dps) > 0.5f) {
                    current_angle += fabsf(dps * dt);
                }
            }
        }

        float error = target_angle - current_angle;

        int16_t current_speed = (int16_t)(error * KP);

        if (current_speed > base_speed)
            current_speed = base_speed;
        if (current_speed < MIN_SPEED)
            current_speed = MIN_SPEED;

        if (angle_deg > 0) {
            mb_drive(current_speed, -current_speed);
        } else {
            mb_drive(-current_speed, current_speed);
        }

        k_sleep(K_MSEC(5));
    }

    if (angle_deg > 0) {
        mb_drive(-TURNSPEED, TURNSPEED); // reverse
    } else {
        mb_drive(TURNSPEED, -TURNSPEED);
    }
    k_sleep(K_MSEC(52)); // Brake duration (needs tuning)

    mb_drive(0, 0);

    k_sleep(K_MSEC(100));

    LOG_INF("Turned %.1f deg (Target %d)", (double)current_angle, angle_deg);
}

void robot_turn_to_north(void) {
    float current_heading, delta_angle;
    int32_t turn_angle;
    uint32_t start_time = k_uptime_get_32();
    int attempts = 0;

    LOG_INF("Navigating to the north");

    while (true) {
        if ((k_uptime_get_32() - start_time) > TURN_TIMEOUT_MS) {
            LOG_ERR("Timeout reached");
            break;
        }

        if (attempts >= MAX_TURN_ATTEMPTS) {
            LOG_WRN("Max attempts reached");
            break;
        }

        current_heading = robot_calculate_heading();

        delta_angle = ROBOT_HEADING_NORTH - current_heading;
        delta_angle = normalize_angle_diff(delta_angle);

        if (fabsf(delta_angle) <= HEADING_TOLERANCE) {
            LOG_INF("Aligned to North: %.1f", (double)current_heading);
            break;
        }

        turn_angle = (int32_t)roundf(delta_angle);

        LOG_INF("Heading: %.1f -> Turning %d", (double)current_heading, turn_angle);

        // TODO: check sign convention of robot_turn vs delta_angle here
        robot_turn(turn_angle);

        k_sleep(K_MSEC(500)); // allow magnetometer to settle
        attempts++;
    }
}

void robot_set_status(robot_status_t status) {
    mb_leds_off();
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
}

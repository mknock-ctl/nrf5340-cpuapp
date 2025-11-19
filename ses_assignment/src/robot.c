#include "robot.h"
#include "lis3mdl.h"
#include <errno.h>
#include <math.h>
#include <mergebot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(robot, LOG_LEVEL_DBG);

float robot_normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

float robot_calculate_heading(void)
{
    lis3mdl_data_t mag_data;
    
    if (lis3mdl_read_mag(&mag_data)) {
        LOG_ERR("Failed to read magnetometer");
        return -1.0f;
    }

    int16_t mag_x = mag_data.x - MAG_OFFSET_X;
    int16_t mag_y = mag_data.y - MAG_OFFSET_Y;

    float heading_rad = atan2f((float)mag_y, (float)mag_x);
    float heading_deg = heading_rad * (180.0f / (float) M_PI);

    heading_deg = 90.0f - heading_deg;

    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    while (heading_deg < 0.0f) heading_deg += 360.0f;

    heading_deg += HEADING_OFFSET_DEG;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    while (heading_deg < 0.0f) heading_deg += 360.0f;

    LOG_DBG("Heading: %.1f° (raw mag X=%d Y=%d Z=%d)",
            (double) heading_deg, mag_data.x, mag_data.y, mag_data.z);

    return heading_deg;
}

void robot_move(int32_t distance_mm) { LOG_INF("move"); }

void robot_turn(int32_t angle_deg) {    
    int32_t turn_time_ms = (int32_t)(fabsf((float)angle_deg) * MS_PER_DEGREE);
    LOG_INF("Turning %d degrees for %d", angle_deg, turn_time_ms);
    
    if (angle_deg > 0) {
        // right (clockwise)
        mb_drive(TURNSPEED, -TURNSPEED);
    } else if (angle_deg < 0) {
        // left (counter-clockwise)
        mb_drive(-TURNSPEED, TURNSPEED);
    } else {
        LOG_DBG("Turn angle is 0, skipping");
        return;
    }
    
    k_sleep(K_MSEC(turn_time_ms));
    
    mb_drive(0, 0);
    k_sleep(K_MSEC(50));
    
    LOG_DBG("Turn complete: %d degrees in %d ms", angle_deg, turn_time_ms);
}

void robot_turn_to_north(void) { 
    float current_heading, delta_angle;
    int32_t turn_angle;
    uint32_t start_time = k_uptime_get_32();
    int attempts = 0;

    LOG_INF("Turning to north (0)...");

    do {
        if ((k_uptime_get_32() - start_time) > TURN_TIMEOUT_MS) {
            LOG_ERR("Turn to north timeout");
            return;
        }

        if (attempts >= MAX_TURN_ATTEMPTS) {
            LOG_ERR("Turn to north max attempts exceeded");
            return;
        }

        current_heading = robot_calculate_heading();
        if (current_heading < 0.0f) return;

        delta_angle = 0.0f - current_heading;
        delta_angle = robot_normalize_angle(delta_angle);

        if (fabsf(delta_angle) <= HEADING_TOLERANCE) {
            LOG_INF("Reached north: %.1f° (within ±%.1f°)",
                    (double)current_heading, (double) HEADING_TOLERANCE);
            return;
        }

        turn_angle = (int32_t)roundf(delta_angle);
        LOG_INF("Attempt %d: heading=%d deg, turning %d deg",
            attempts + 1, (int)current_heading, turn_angle);
        
        robot_turn(turn_angle);

        k_sleep(K_MSEC(100));
        attempts++;

    } while (true);
}

void robot_set_status(robot_status_t status)
{
    const char *status_str[] = {
        [STATUS_OK]    = "OK",
        [STATUS_FAST]  = "Too fast (downhill)",
        [STATUS_SLOW]  = "Too slow (uphill)",
        [STATUS_CRASH] = "Crash detected",
    };

    if (status < ARRAY_SIZE(status_str)) {
        LOG_INF("Status: %s", status_str[status]);
    }

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
        mb_led_toggle(MB_LED_G);  /* Red + Green = Yellow */
        break;
    case STATUS_CRASH:
        mb_led_toggle(MB_LED_R);
        break;
    }
}

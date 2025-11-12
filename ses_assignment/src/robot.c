#include "robot.h"
#include <stdint.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(robot, LOG_LEVEL_DBG);

void robot_move(int32_t distance_mm) { LOG_INF("move"); }

void robot_turn(int32_t angle_deg) { LOG_INF("turn"); }

void robot_turn_to_north(void) { LOG_INF("turning to north"); }

void robot_set_status(robot_status_t status) {
    const char *status_str[] = {[STATUS_OK] = "OK",
                                [STATUS_FAST] = "Too fast",
                                [STATUS_SLOW] = "Too slow",
                                [STATUS_CRASH] = "Crash"};

    LOG_INF("status: %s", status_str[status]);
}

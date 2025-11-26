#ifndef ROBOT_H
#define ROBOT_H

#include "ses_assignment.h"
#include <zephyr/kernel.h>
#include <stdint.h>

typedef enum {
    IMU_MODE_OFF,
    IMU_MODE_TAP,
    IMU_MODE_CRASH
} robot_imu_mode_t;

typedef enum {
    STATUS_OK,
    STATUS_FAST,
    STATUS_SLOW,
    STATUS_CRASH,
} robot_status_t;

int robot_init(void);
void robot_set_imu_mode(robot_imu_mode_t new_mode);

void robot_move(int32_t distance_mm);
void robot_turn(int32_t angle);
void robot_turn_to_north(void);
float robot_calculate_heading(void);

void robot_set_status(robot_status_t code);

#endif
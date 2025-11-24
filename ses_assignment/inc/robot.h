#ifndef ROBOT_H
#define ROBOT_H

#include "ses_assignment.h"
#include <stdint.h>

void robot_move(int32_t distance_mm);

void robot_turn(int32_t angle);

void robot_turn_to_north(void);

float robot_calculate_heading(void);

typedef enum {
    STATUS_OK,
    STATUS_FAST,
    STATUS_SLOW,
    STATUS_CRASH,
} robot_status_t;

void robot_set_status(robot_status_t code);

#endif

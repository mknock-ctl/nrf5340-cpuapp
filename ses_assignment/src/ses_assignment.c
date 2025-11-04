/**
 * @file ses_assignment.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Stubs for SES assignment
 * @date 2025-10-01
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ses_assignment.h"
#include <mergebot.h>

LOG_MODULE_REGISTER(ses_assignment, LOG_LEVEL_DBG);

void move(int32_t distance) { LOG_ERR("move is not yet implemented"); }

void turn(int32_t angle) { LOG_ERR("turn is not yet implemented"); }

void turn_to_north(void) { LOG_ERR("turn_to_north is not yet implemented"); }

void wait_for_double_tap(void) { LOG_ERR("wait_for_double_tap is not yet implemented"); }

void update_status(ses_status_code code) { LOG_ERR("set_status_led is not yet implemented"); }

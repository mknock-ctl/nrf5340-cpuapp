/**
 * @file ses_assignment.h
 * @brief Common macros & definitions for this SES assignment
 * @date 2025-10-01
 */

#ifndef SES_ASSIGNMENT_H
#define SES_ASSIGNMENT_H

#include <stdint.h>

/// @brief Try executing `test`, if it returns a non-zero value, log the error and return from the
/// current function
#define TRY(test)                                                                                  \
    do {                                                                                           \
        if ((test)) {                                                                              \
            LOG_ERR("%s failed at %s:%d", #test, __FILE__, __LINE__);                              \
            return;                                                                                \
        }                                                                                          \
    } while (0)

/// @brief Try executing `test`, if it returns a non-zero value, return that value from the current
/// function
#define TRY_ERR(err_t, test)                                                                       \
    do {                                                                                           \
        err_t err = (test);                                                                        \
        if (err)                                                                                   \
            return err;                                                                            \
    } while (0)

/// @brief Mark a code location as unreachable, triggering a fatal error if reached
#define UNREACHABLE()                                                                              \
    do {                                                                                           \
        __ASSERT(false, "Reached unreachable code at %s:%d", __FILE__, __LINE__);                  \
    } while (0)

/**
 * @brief Move the robot a certain distance, stop afterwards
 *
 * @param distance Target distance in millimeter, negative distances indicate moving backwards.
 */
void robot_move(int32_t distance_mm);

/**
 * @brief Turn the robot a certain angle, stop afterwards
 *
 * @param angle Target angle in degrees, negative angles indicate clockwise.
 */
void robot_turn(int32_t angle);

/**
 * @brief Make the robot face North
 *
 * @details Turn the robot until its front faces North using the shortest path.
 *          For example, if the robot is currently facing East it moves counterclockwise.
 *          Only if it is perfectly facing South we do not care about the turn direction.
 *
 * @note This function should finish within 30 seconds.
 */
void robot_turn_to_north(void);

/**
 * @brief Block execution of the current thread until a double tap is detected.
 */
//void wait_for_double_tap(void);

/**
 * @brief Possible codes for the status LED
 * @note Just a suggestion, can be modified if needed
 */
typedef enum {
    STATUS_OK,      //!< Everything running ok
    STATUS_FAST, //!< Downhill loss of traction
    STATUS_SLOW, //!< Uphill loss of traction
    STATUS_CRASH,   //<! Crashed into something
} robot_status_t;

typedef struct k_work tap_work;

/**
 * @brief Update the status LED
 *
 * @param code New status code
 */
void robot_set_status(robot_status_t code);

#endif // SES_ASSIGNMENT_H

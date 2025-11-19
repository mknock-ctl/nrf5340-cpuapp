#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>

#define SPEED 100 // MAX 300
#define TURNSPEED 100
#define MS_PER_DEGREE 8

// calibration
#define MAG_OFFSET_X        0      
#define MAG_OFFSET_Y        0      
#define HEADING_OFFSET_DEG  0.0f  

// Navigation parameters 
#define HEADING_TOLERANCE   5.0f  
#define TURN_TIMEOUT_MS     30000  // Maximum time for turn_to_north 
#define MAX_TURN_ATTEMPTS   2     // Safety limit on corrections 

#define ROBOT_HEADING_NORTH     0.0f
#define ROBOT_HEADING_TOLERANCE 5.0f
#define ROBOT_TURN_TIMEOUT_MS   30000

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

#endif

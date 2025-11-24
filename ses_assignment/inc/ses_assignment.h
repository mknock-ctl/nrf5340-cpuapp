#ifndef SES_ASSIGNMENT_H
#define SES_ASSIGNMENT_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Constants */
#define CALIBRATION_RESET false
#define TAP_TIMEOUT_MS 100

#define SPEED 100
#define TURNSPEED 100

#define MAG_OFFSET_X 0
#define MAG_OFFSET_Y 0
#define HEADING_OFFSET_DEG 0.0f

#define HEADING_TOLERANCE 5.0f
#define TURN_TIMEOUT_MS 30000
#define MAX_TURN_ATTEMPTS 3

#define ROBOT_HEADING_NORTH 0.0f

#define VCAP_MAX 3000
#define VCAP_MIN 600
#define THRESHOLD_HIGH 66
#define THRESHOLD_MED 33

/* Macros */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TRY(test)                                                                                  \
    do {                                                                                           \
        if ((test)) {                                                                              \
            LOG_ERR("%s failed at %s:%d", #test, __FILE__, __LINE__);                              \
            return;                                                                                \
        }                                                                                          \
    } while (0)

#define TRY_ERR(err_t, test)                                                                       \
    do {                                                                                           \
        err_t err = (test);                                                                        \
        if (err)                                                                                   \
            return err;                                                                            \
    } while (0)

#define CONFIGURE_REGS(write_func, ...)                                                            \
    struct {                                                                                       \
        uint8_t reg;                                                                               \
        uint8_t val;                                                                               \
    } config[] = {__VA_ARGS__};                                                                    \
    for (size_t i = 0; i < ARRAY_SIZE(config); i++) {                                              \
        TRY_ERR(int, write_func(config[i].reg, config[i].val));                                    \
    }

#define INIT_CHECK(func)                                                                           \
    do {                                                                                           \
        int _ret = (func);                                                                         \
        if (_ret != 0 && _ret != 0 /* MB_SUCCESS */) {                                             \
            LOG_ERR("%s failed: %d", #func, _ret);                                                 \
            return _ret;                                                                           \
        }                                                                                          \
    } while (0)

#define UNREACHABLE()                                                                              \
    do {                                                                                           \
        __ASSERT(false, "Unreachable code at %s:%d", __FILE__, __LINE__);                          \
    } while (0)

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#define RAD_TO_DEG(rad) ((rad) * (180.0 / M_PI))

#endif

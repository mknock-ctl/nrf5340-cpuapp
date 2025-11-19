
#ifndef SES_ASSIGNMENT_H
#define SES_ASSIGNMENT_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/logging/log.h>

#define TRY(test)                                                       \
    do {                                                                \
        if ((test)) {                                                   \
            LOG_ERR("%s failed at %s:%d", #test, __FILE__, __LINE__);   \
            return;                                                     \
        }                                                               \
    } while (0)

#define TRY_ERR(err_t, test)                                                                       \
    do {                                                                                           \
        err_t err = (test);                                                                        \
        if (err)  {                                                                                \
            LOG_ERR("%s failed: %d", #test, err);                                                  \
            return err;                                                                            \
        }                                                                                          \
    } while (0)

#define INIT_CHECK(func)                                        \
    do {                                                        \
        int _ret = (func);                                      \
        if (_ret != 0 && _ret != MB_SUCCESS) {                  \
            LOG_ERR("%s failed: %d", #func, _ret);              \
            return _ret;                                        \
        }                                                       \
    } while (0)

#define UNREACHABLE()                                                   \
    do {                                                                \
        __ASSERT(false, "Unreachable code at %s:%d",                    \
                 __FILE__, __LINE__);                                   \
    } while (0)

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))

#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))

#endif
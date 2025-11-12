#ifndef POWER_H
#define POWER_H

#include <stdint.h>

typedef enum {
    POWER_HIGH,
    POWER_MEDIUM,
    POWER_LOW
} power_level_t;

power_level_t power_get_level(void);
void power_update_indicator(void);

#endif
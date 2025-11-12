#include "power.h"
#include <stdbool.h>
#include <mergebot.h>

#define VCAP_MAX 3000
#define VCAP_MIN 600
#define THRESHOLD_HIGH 66
#define THRESHOLD_MED 33

static power_level_t last_level = POWER_HIGH + 1;

static int vcap_to_percent(int vcap) {
    if (vcap >= VCAP_MAX) {
        return 100;
    }
    if (vcap <= VCAP_MIN) {
        return 0;
    }
    return (100 * (vcap - VCAP_MIN)) / (VCAP_MAX - VCAP_MIN);
}

static power_level_t percent_to_level(int percent) {
    if (percent > THRESHOLD_HIGH) {
        return POWER_HIGH;
    }
    if (percent > THRESHOLD_MED) {
        return POWER_MEDIUM;
    }
    return POWER_LOW;
}

power_level_t power_get_level(void) {
    int vcap = mb_measure_vcap();
    int percent = vcap_to_percent(vcap);
    return percent_to_level(percent);
}

void power_update_indicator(void) {
    power_level_t level = power_get_level();

    if (level == last_level) {
        return;
    }

    mb_leds_off();

    switch (level) {
    case POWER_HIGH:
        mb_led_toggle(MB_LED_G);
        break;
    case POWER_MEDIUM:
        mb_led_toggle(MB_LED_R);
        mb_led_toggle(MB_LED_G);
        break;
    case POWER_LOW:
        mb_led_toggle(MB_LED_R);
        break;
    }

    last_level = level;
}

#include "robot/power.h"
#include "ses_assignment.h"
#include <mergebot.h>
#include <stdbool.h>

// https://distrinet.pages.gitlab.kuleuven.be/taskforces/nes/freebot/documentation/events/eums03-embedded-ex2.html
#define V2E(v) (12 * (v) * (v) / 100000) /* CV^2/2 */
#define VCAP_MAX 3000                    /* mV */
#define VCAP_MIN 600                     /* mV */
#define E_MAX V2E(VCAP_MAX)              /* J */
#define E_MIN V2E(VCAP_MIN)              /* J */

static uint8_t vcap_to_percent(int vcap) {
    uint32_t energy = V2E(vcap);
    energy = MAX(MIN(energy, E_MAX), E_MIN);
    uint8_t level = (energy - E_MIN) * 100 / (E_MAX - E_MIN);
    return level;
}

uint8_t power_percentage(void) {
    int16_t vcap = mb_measure_vcap();
    if (vcap < 0) { // Read error => return 0%
        return 0;
    }

    return vcap_to_percent(vcap);
}

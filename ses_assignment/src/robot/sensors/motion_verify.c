
#include "robot/sensors/sensor_base.h"
#include "robot/sensors/motion_verify.h"
#include "robot/sensors/int1_gpio.h"
#include "robot/sensors/lsm6dsox.h"
#include "robot.h"
#include "mergebot.h"
#include "ses_assignment.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(motion_verify, LOG_LEVEL_DBG);

extern struct gpio_int_handle int1_handle;

#define VELOCITY_COMPARISON_PERIOD_MS 200  /* Compare every 200ms */
#define VELOCITY_THRESHOLD_PERCENT 20.0f   /* 20% difference triggers status change */
#define MIN_VELOCITY_THRESHOLD_MM_S 100.0f /* Ignore velocities below this */

typedef struct motion_sensor {
    sensor_base_t base;

    volatile bool moving_forward;
    motion_status_t current_status;

    int32_t last_encoder_left;
    int32_t last_encoder_right;
    int64_t last_time_ms;

    float accel_velocity_ms;
    int64_t last_accel_time_ms;

    struct k_work motion_verify_work;
    struct k_timer comparison_timer;
} motion_sensor_t;

static motion_sensor_t g_motion = {
    .base = {
        .name = "motion_verify",
        .ops = NULL,
        .gpio = NULL,
        .gpio_handle = &int1_handle
    },
    .moving_forward = false,
    .current_status = MOTION_STATUS_OK,
    .last_encoder_left = 0,
    .last_encoder_right = 0,
    .last_time_ms = 0,
    .accel_velocity_ms = 0.0f,
    .last_accel_time_ms = 0,
};

static inline float accel_raw_to_ms2(int16_t raw) {
    return (float)raw * 0.000598f;
}

static float get_encoder_velocity_mm_s(motion_sensor_t *ms) {
    int32_t current_left, current_right;
    mb_angle(&current_left, &current_right);

    int64_t current_time = k_uptime_get();

    int64_t dt_ms = current_time - ms->last_time_ms;
    if (dt_ms <= 0) {
        /* update encoder snapshot but return 0 to avoid division by zero */
        ms->last_encoder_left = current_left;
        ms->last_encoder_right = current_right;
        ms->last_time_ms = current_time;
        return 0.0f;
    }

    int32_t delta_left = current_left - ms->last_encoder_left;
    int32_t delta_right = current_right - ms->last_encoder_right;
    int32_t avg_delta_ticks = (delta_left + delta_right) / 2;

    float circumference_mm = M_PI * WHEEL_DIAMETER_MM;
    float distance_mm = ((float)avg_delta_ticks / (float)TICKS_PER_REV) * circumference_mm;

    float velocity_mm_s = (distance_mm / (float)dt_ms) * 1000.0f;

    ms->last_encoder_left = current_left;
    ms->last_encoder_right = current_right;
    ms->last_time_ms = current_time;

    return ms->moving_forward ? velocity_mm_s : -velocity_mm_s;
}

static void motion_verify_work_handler(struct k_work *work) {
    motion_sensor_t *ms = CONTAINER_OF(work, motion_sensor_t, motion_verify_work);
    if (!ms->base.active) return;

    int16_t accel_x, accel_y, accel_z;
    if (lsm6dsox_read_accel_raw(&accel_x, &accel_y, &accel_z) != 0) {
        return;
    }

    float accel_ms2 = accel_raw_to_ms2(accel_x);

    int64_t current_time = k_uptime_get();
    if (ms->last_accel_time_ms == 0) {
        ms->last_accel_time_ms = current_time;
        return;
    }

    float dt_s = (current_time - ms->last_accel_time_ms) / 1000.0f;
    ms->last_accel_time_ms = current_time;

    /* Integrate acceleration to get velocity change (m/s) */
    ms->accel_velocity_ms += accel_ms2 * dt_s;
}

static void comparison_timer_callback(struct k_timer *timer) {
    motion_sensor_t *ms = CONTAINER_OF(timer, motion_sensor_t, comparison_timer);
    if (!ms->base.active) return;

    float encoder_velocity_mm_s = get_encoder_velocity_mm_s(ms);
    float accel_velocity_mm_s = ms->accel_velocity_ms * 1000.0f; /* m/s -> mm/s */

    if (fabsf(encoder_velocity_mm_s) < MIN_VELOCITY_THRESHOLD_MM_S) {
        /* too slow to compare meaningfully */
        if (ms->current_status != MOTION_STATUS_OK) {
            ms->current_status = MOTION_STATUS_OK;
            robot_set_status(STATUS_OK);
        }
        return;
    }

    float velocity_diff = accel_velocity_mm_s - encoder_velocity_mm_s;
    float percent_diff = (velocity_diff / fabsf(encoder_velocity_mm_s)) * 100.0f;

    LOG_DBG("Encoder: %.1f mm/s, Accel: %.1f mm/s, Diff: %.1f%%",
            (double)encoder_velocity_mm_s, (double)accel_velocity_mm_s, (double)percent_diff);

    motion_status_t new_status = MOTION_STATUS_OK;
    if (percent_diff > VELOCITY_THRESHOLD_PERCENT) {
        new_status = MOTION_STATUS_FAST;
    } else if (percent_diff < -VELOCITY_THRESHOLD_PERCENT) {
        new_status = MOTION_STATUS_SLOW;
    }

    if (new_status != ms->current_status) {
        ms->current_status = new_status;
        robot_set_status((robot_status_t)ms->current_status);
        switch (new_status) {
        case MOTION_STATUS_OK:
            LOG_INF("Motion: OK (good traction)");
            break;
        case MOTION_STATUS_FAST:
            LOG_WRN("Motion: FAST (downhill/slip) - Accel %.1f%% faster", (double)percent_diff);
            break;
        case MOTION_STATUS_SLOW:
            LOG_WRN("Motion: SLOW (uphill/resistance) - Accel %.1f%% slower", (double)fabsf(percent_diff));
            break;
        default:
            break;
        }
    }
}

static void accel_dataready_isr(gpio_pin_t pin, void *user_data) {
    ARG_UNUSED(pin);
    ARG_UNUSED(user_data);

    if (!g_motion.base.active) return;

    k_work_submit(&g_motion.motion_verify_work);
}

static int motion_init(sensor_base_t *s) {
    motion_sensor_t *ms = CONTAINER_OF(s, motion_sensor_t, base);

    k_work_init(&ms->motion_verify_work, motion_verify_work_handler);
    k_timer_init(&ms->comparison_timer, comparison_timer_callback, NULL);

    int ret = gpio_int_register_callback(ms->base.gpio_handle, accel_dataready_isr, NULL);
    if (ret < 0 && ret != -ENOMEM) {
        LOG_ERR("Failed to register callback: %d", ret);
        return ret;
    }

    ret = lsm6dsox_route_int1(INT1_DRDY_XL, true);
    if (ret != 0) {
        LOG_ERR("Failed to route accel DRDY to INT1: %d", ret);
        if (ret != -ENOMEM) {
            gpio_int_unregister_callback(ms->base.gpio_handle, accel_dataready_isr);
        }
        return ret;
    }

    ms->current_status = MOTION_STATUS_OK;
    ms->base.active = false;
    LOG_INF("Motion verification initialized");
    return 0;
}

static int motion_deinit(sensor_base_t *s) {
    motion_sensor_t *ms = CONTAINER_OF(s, motion_sensor_t, base);
    ms->base.active = false;
    k_timer_stop(&ms->comparison_timer);

    lsm6dsox_route_int1(INT1_DRDY_XL, false);
    gpio_int_unregister_callback(ms->base.gpio_handle, accel_dataready_isr);

    LOG_INF("Motion verification deinitialized");
    return 0;
}

static int motion_start(sensor_base_t *s, bool forward) {
    motion_sensor_t *ms = CONTAINER_OF(s, motion_sensor_t, base);
    LOG_INF("Starting motion verification (forward=%d)", forward ? 1 : 0);

    ms->moving_forward = forward;

    mb_angle(&ms->last_encoder_left, &ms->last_encoder_right);
    ms->last_time_ms = k_uptime_get();

    ms->accel_velocity_ms = 0.0f;
    ms->last_accel_time_ms = 0;

    ms->current_status = MOTION_STATUS_OK;
    robot_set_status(STATUS_OK);

    k_timer_start(&ms->comparison_timer,
                  K_MSEC(VELOCITY_COMPARISON_PERIOD_MS),
                  K_MSEC(VELOCITY_COMPARISON_PERIOD_MS));

    ms->base.active = true;
    return 0;
}

static void motion_stop(sensor_base_t *s) {
    motion_sensor_t *ms = CONTAINER_OF(s, motion_sensor_t, base);
    ms->base.active = false;
    k_timer_stop(&ms->comparison_timer);
    LOG_INF("Motion verification stopped");
}

SENSOR_BASE_DEFINE_OPS(motion) = {
    .init = motion_init,
    .deinit = motion_deinit,
    .start = motion_start,
    .stop = motion_stop,
    .irq_enable = NULL,
};

int motion_verify_init(void) {
    g_motion.base.ops = &motion_ops;
    sensor_base_register(&g_motion.base);
    return sensor_base_init(&g_motion.base);
}

void motion_verify_start(bool forward) {
    sensor_base_start(&g_motion.base, forward);
}

void motion_verify_stop(void) {
    sensor_base_stop(&g_motion.base);
}

motion_status_t motion_verify_get_status(void) {
    return g_motion.current_status;
}

int motion_verify_deinit(void) {
    sensor_base_unregister(&g_motion.base);
    return sensor_base_deinit(&g_motion.base);
}

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

#define CHECK_INTERVAL_MS 150
#define THRESHOLD_PERCENT 25.0f
#define MIN_VEL_MM_S 80.0f
#define ACCEL_TO_MS2 0.000598f

typedef struct {
    sensor_base_t base;
    bool fwd;
    motion_status_t status;
    
    int32_t enc_l, enc_r;
    int64_t last_time;
    
    float grav_cal;
    int32_t acc_sum;
    uint16_t acc_cnt;
    
    struct k_work accel_work;
    struct k_timer timer;
} motion_t;

static motion_t g_motion = {
    .base = {.name = "motion_verify", .gpio_handle = &int1_handle},
    .fwd = false,
    .status = MOTION_STATUS_OK,
};

static void accel_work_handler(struct k_work *work) {
    motion_t *m = CONTAINER_OF(work, motion_t, accel_work);
    if (!m->base.active) return;
    
    int16_t x, y, z;
    if (lsm6dsox_read_accel_raw(&x, &y, &z) == 0) {
        LOG_DBG("Accel read: x=%d y=%d z=%d", x, y, z);
        m->acc_sum += x;
        m->acc_cnt++;
    }
}

static void timer_cb(struct k_timer *t) {
    motion_t *m = CONTAINER_OF(t, motion_t, timer);
    if (!m->base.active || m->acc_cnt == 0) return;

    int32_t el, er;
    mb_angle(&el, &er);
    int64_t now = k_uptime_get();
    int64_t dt = now - m->last_time;
    
    if (dt <= 0) {
        m->enc_l = el;
        m->enc_r = er;
        m->last_time = now;
        m->acc_sum = 0;
        m->acc_cnt = 0;
        return;
    }

    int32_t ticks = ((el - m->enc_l) + (er - m->enc_r)) / 2;
    float dist = ((float)ticks / TICKS_PER_REV) * M_PI * WHEEL_DIAMETER_MM;
    float v_enc = (dist / dt) * 1000.0f * (m->fwd ? 1.0f : -1.0f);

    float a_avg = ((float)m->acc_sum / m->acc_cnt - m->grav_cal) * ACCEL_TO_MS2;
    float v_acc = a_avg * (dt / 1000.0f) * 1000.0f;

    m->enc_l = el;
    m->enc_r = er;
    m->last_time = now;
    m->acc_sum = 0;
    m->acc_cnt = 0;

    if (fabsf(v_enc) < MIN_VEL_MM_S) {
        if (m->status != MOTION_STATUS_OK) {
            m->status = MOTION_STATUS_OK;
            robot_set_status(STATUS_OK);
        }
        return;
    }

    float diff = ((v_acc - v_enc) / fabsf(v_enc)) * 100.0f;

    motion_status_t new_st = MOTION_STATUS_OK;
    if (diff > THRESHOLD_PERCENT) new_st = MOTION_STATUS_FAST;
    else if (diff < -THRESHOLD_PERCENT) new_st = MOTION_STATUS_SLOW;

    if (new_st != m->status) {
        m->status = new_st;
        robot_set_status((robot_status_t)new_st);
    }
}

static void isr(gpio_pin_t pin, void *ud) {
    ARG_UNUSED(pin);
    ARG_UNUSED(ud);
    if (g_motion.base.active) {
        LOG_DBG("Motion verify ISR triggered");
        k_work_submit(&g_motion.accel_work);
    }
}

static int init(sensor_base_t *s) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    
    k_work_init(&m->accel_work, accel_work_handler);
    k_timer_init(&m->timer, timer_cb, NULL);

    int r = gpio_int_register_callback(m->base.gpio_handle, isr, NULL);
    if (r < 0 && r != -ENOMEM) return r;

    r = lsm6dsox_route_int1(INT1_DRDY_XL, true);
    if (r != 0) {
        if (r != -ENOMEM) gpio_int_unregister_callback(m->base.gpio_handle, isr);
        return r;
    }

    m->status = MOTION_STATUS_OK;
    m->base.active = false;
    return 0;
}

static int deinit(sensor_base_t *s) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    m->base.active = false;
    k_timer_stop(&m->timer);
    lsm6dsox_route_int1(INT1_DRDY_XL, false);
    gpio_int_unregister_callback(m->base.gpio_handle, isr);
    return 0;
}

static int start(sensor_base_t *s, bool forward) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    
    int e = lsm6dsox_accel_set_odr(true);
    if (e) return e;

    lsm6dsox_clear_interrupts();
    
    int16_t x, y, z;
    int32_t sum = 0;
    for (int i = 0; i < 20; i++) {
        if (lsm6dsox_read_accel_raw(&x, &y, &z) == 0) sum += x;
        k_sleep(K_MSEC(5));
    }
    m->grav_cal = sum / 20.0f;

    m->fwd = forward;
    mb_angle(&m->enc_l, &m->enc_r);
    m->last_time = k_uptime_get();
    m->acc_sum = 0;
    m->acc_cnt = 0;
    m->status = MOTION_STATUS_OK;
    robot_set_status(STATUS_OK);

    k_timer_start(&m->timer, K_MSEC(CHECK_INTERVAL_MS), K_MSEC(CHECK_INTERVAL_MS));
    m->base.active = true;
    return 0;
}

static void stop(sensor_base_t *s) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    m->base.active = false;
    k_timer_stop(&m->timer);
}

SENSOR_BASE_DEFINE_OPS(motion) = {
    .init = init,
    .deinit = deinit,
    .start = start,
    .stop = stop,
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
    return g_motion.status;
}

int motion_verify_deinit(void) {
    sensor_base_unregister(&g_motion.base);
    return sensor_base_deinit(&g_motion.base);
}
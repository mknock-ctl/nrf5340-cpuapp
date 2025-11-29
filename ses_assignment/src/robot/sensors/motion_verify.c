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

#define CHECK_INTERVAL_MS 200
#define THRESHOLD_PERCENT 5.0f
#define MIN_VEL_MM_S 50.0f
#define ACCEL_TO_MS2 0.000598f
#define ENCODER_TRUST 0.85f  // 85% encoder, 15% accelerometer
#define ACCEL_TRUST (1.0f - ENCODER_TRUST)
#define HPF_ALPHA 0.95f

typedef struct {
    sensor_base_t base;
    bool fwd;
    motion_status_t status;
    
    int32_t enc_l, enc_r;
    int64_t last_time;
    
    float grav_cal;
    int32_t acc_sum;
    uint16_t acc_cnt;

    float filtered_velocity;
    float acc_prev_raw;
    float acc_prev_filtered;
    
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
        m->acc_sum += y;
        m->acc_cnt++;
    }
}

static void timer_cb(struct k_timer *t) {
    motion_t *m = CONTAINER_OF(t, motion_t, timer);
    
    if (!m->base.active) {
        return;
    }
    
    if (m->acc_cnt == 0) {
        return;
    }

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

    float acc_raw = (float)m->acc_sum / m->acc_cnt;
    float acc_filtered = HPF_ALPHA * (m->acc_prev_filtered + acc_raw - m->acc_prev_raw);
    m->acc_prev_raw = acc_raw;
    m->acc_prev_filtered = acc_filtered;

    float a_ms2 = acc_filtered * ACCEL_TO_MS2;
    float dt_sec = dt / 1000.0f;
    float dv_accel = a_ms2 * dt_sec * 1000.0f;

     // This prevents unbounded drift while still detecting anomalies
    float v_accel_integrated = m->filtered_velocity + dv_accel;
    m->filtered_velocity = ENCODER_TRUST * v_enc + ACCEL_TRUST * v_accel_integrated;

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
        m->filtered_velocity = 0.0f;
        return;
    }

    float diff = ((m->filtered_velocity - v_enc) / fabsf(v_enc)) * 100.0f;
    LOG_INF("v_enc: %.2f mm/s, v_filt: %.2f mm/s, diff: %.2f%%, acc: %.2f m/s²",
            (double)v_enc, (double)m->filtered_velocity, (double)diff, (double)a_ms2);

    motion_status_t new_st = MOTION_STATUS_OK;
    if (diff > THRESHOLD_PERCENT) {
        new_st = MOTION_STATUS_FAST;
    } else if (diff < -THRESHOLD_PERCENT) {
        new_st = MOTION_STATUS_SLOW;
    }

    if (new_st != m->status) {
        m->status = new_st;
        robot_set_status((robot_status_t)new_st);
    }
}

static void isr(gpio_pin_t pin, void *ud) {
    ARG_UNUSED(pin);
    ARG_UNUSED(ud);
    LOG_DBG("Motion verify ISR triggered");
    
    if (g_motion.base.active) {        
        k_work_submit(&g_motion.accel_work);
    }
}

static int init(sensor_base_t *s) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    
    k_work_init(&m->accel_work, accel_work_handler);
    k_timer_init(&m->timer, timer_cb, NULL);

    int r = gpio_int_register_callback(m->base.gpio_handle, isr, NULL);
    if (r < 0 && r != -ENOMEM) {
        LOG_ERR("Failed to register GPIO callback: %d", r);
        return r;
    }

    m->status = MOTION_STATUS_OK;
    m->base.active = false;
    
    LOG_INF("Motion verify initialized");
    return 0;
}

static int deinit(sensor_base_t *s) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    m->base.active = false;
    k_timer_stop(&m->timer);
    lsm6dsox_route_int1(INT1_DRDY_XL, false);
    gpio_int_unregister_callback(m->base.gpio_handle, isr);
    LOG_INF("Motion verify deinitialized");
    return 0;
}

static int start(sensor_base_t *s, bool forward) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    uint8_t md1_cfg;
    lsm6dsox_read_reg(LSM6DSOX_MD1_CFG, &md1_cfg);
    LOG_INF("MD1_CFG = 0x%02X", md1_cfg);
    
    if ((md1_cfg & INT1_DRDY_XL) == 0) {
        LOG_ERR("INT1_DRDY_XL not set in MD1_CFG!");
        return -EIO;
    }
    
    k_sleep(K_MSEC(50));
    
    LOG_INF("Calibrating gravity...");
    int16_t x, y, z;
    int32_t sum = 0;
    int valid = 0;
    
    for (int i = 0; i < 100; i++) {
        if (lsm6dsox_read_accel_raw(&x, &y, &z) == 0) {
            sum += y;
            valid++;
        }
        k_sleep(K_MSEC(5));
    }
    
    if (valid < 50) {
        LOG_ERR("Insufficient calibration samples: %d/20", valid);
        return -EIO;
    }
    
    m->grav_cal = (float)sum / valid;
    LOG_INF("Gravity cal: %.2f (%d samples)", (double)m->grav_cal, valid);

    m->fwd = forward;
    mb_angle(&m->enc_l, &m->enc_r);
    m->last_time = k_uptime_get();
    m->acc_sum = 0;
    m->acc_cnt = 0;
    m->filtered_velocity = 0.0f;
    m->acc_prev_raw = m->grav_cal;
    m->acc_prev_filtered = 0.0f;
    m->status = MOTION_STATUS_OK;
    robot_set_status(STATUS_OK);

    m->base.active = true;
    k_timer_start(&m->timer, K_MSEC(CHECK_INTERVAL_MS), K_MSEC(CHECK_INTERVAL_MS));
    
    LOG_INF("Motion verify started successfully");
    return 0;
}

static void stop(sensor_base_t *s) {
    motion_t *m = CONTAINER_OF(s, motion_t, base);
    m->base.active = false;
    k_timer_stop(&m->timer);
    lsm6dsox_route_int1(INT1_DRDY_XL, false);
    LOG_INF("Motion verify stopped");
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
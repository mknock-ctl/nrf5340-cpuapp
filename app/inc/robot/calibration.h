#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

extern float g_gyro_bias_z;

extern float g_mag_offset_x;
extern float g_mag_offset_y;
extern float g_mag_scale_x, g_mag_scale_y;

extern float g_ticks_per_mm;

extern float g_motor_left_scale;  // Scale factor for left motor (1.0 = baseline)
extern float g_motor_right_scale; // Scale factor for right motor (1.0 = baseline)
extern float g_turn_cw_scale;     // Turn rate scale for CW (1.0 = baseline)
extern float g_turn_ccw_scale;    // Turn rate scale for CCW (1.0 = baseline)
extern float g_cw_stop_ahead;     // Degrees to stop ahead for CW turns
extern float g_ccw_stop_ahead;    // Degrees to stop ahead for CCW turns
extern float g_cw_brake_scale;    // Brake strength multiplier for CW
extern float g_ccw_brake_scale;   // Brake strength multiplier for CCW

int calibration_init(bool reset);
void cal_encoders(uint32_t duration_ms, void (*drv)(int16_t, int16_t), void (*led)(int));
void cal_motor_drive_asymmetry(void);
bool calibration_needed(void);
void calibration_sequence(int16_t speed, void (*drive_func)(int16_t, int16_t),
                          void (*led_func)(int));

#endif

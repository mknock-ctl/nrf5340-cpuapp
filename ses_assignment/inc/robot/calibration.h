#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

extern float g_gyro_bias_z;
extern float g_ms_per_degree;

extern float g_mag_offset_x;
extern float g_mag_offset_y;
extern float g_mag_scale_x, g_mag_scale_y;

int calibration_init(bool reset);
bool calibration_needed(void);
void calibration_sequence(int16_t speed, void (*drive_func)(int16_t, int16_t),
                           void (*led_func)(int));

#endif

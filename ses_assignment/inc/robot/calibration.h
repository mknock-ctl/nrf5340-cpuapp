#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

extern float g_gyro_bias_z;
extern float g_ms_per_degree;

int calibration_init(bool reset);
bool calibration_needed(void);
void calibrate_gyro_offset(void);
float calibration_sequence(int16_t speed, void (*drive_func)(int16_t, int16_t),
                           void (*led_func)(int));

#endif

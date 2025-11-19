/**
 * LIS3MDL magnetometer driver interface
 */

#ifndef LIS3MDL_H
#define LIS3MDL_H

#include <stdint.h>
#include <zephyr/devicetree.h>
#define LIS3MDL_NODE DT_NODELABEL(lis3mdl)

#define LIS3MDL_WHO_AM_I        0x0F
#define LIS3MDL_CTRL_REG1       0x20
#define LIS3MDL_CTRL_REG2       0x21
#define LIS3MDL_CTRL_REG3       0x22
#define LIS3MDL_CTRL_REG4       0x23
#define LIS3MDL_CTRL_REG5       0x24
#define LIS3MDL_STATUS_REG      0x27
#define LIS3MDL_OUT_X_L         0x28
#define LIS3MDL_OUT_X_H         0x29
#define LIS3MDL_OUT_Y_L         0x2A
#define LIS3MDL_OUT_Y_H         0x2B
#define LIS3MDL_OUT_Z_L         0x2C
#define LIS3MDL_OUT_Z_H         0x2D
#define LIS3MDL_TEMP_OUT_L      0x2E
#define LIS3MDL_TEMP_OUT_H      0x2F

#define LIS3MDL_WHO_AM_I_VALUE 0x3D

#define LIS3MDL_TEMP_EN         (1 << 7)
#define LIS3MDL_OM_HIGH_PERF    (0x03 << 5) // High performance XY mode
#define LIS3MDL_ODR_80HZ        (0x07 << 2) // 80 Hz output data rate

#define LIS3MDL_FS_4_GAUSS      (0x00 << 5)  // 4 gauss full scale 
#define LIS3MDL_FS_8_GAUSS      (0x01 << 5)  // 8 gauss full scale
#define LIS3MDL_FS_12_GAUSS     (0x02 << 5)  // 12 gauss full scale
#define LIS3MDL_FS_16_GAUSS     (0x03 << 5)  // 16 gauss full scale

#define LIS3MDL_MD_CONTINUOUS   0x00         // continuous conversion mode
#define LIS3MDL_MD_SINGLE       0x01         // Single conversion mode 
#define LIS3MDL_MD_POWER_DOWN   0x03         // powerdown mode

#define LIS3MDL_OMZ_HIGH_PERF   (0x03 << 2)  // High performance Z mode 
#define LIS3MDL_BLE_LSB         (0 << 1)     // little endian 

#define LIS3MDL_BDU_CONTINUOUS  (0 << 6)     // Continuous update 
#define LIS3MDL_BDU_BLOCKED     (1 << 6)     // block data update

#define LIS3MDL_SENSITIVITY_4G  6842         // LSB/gauss at 4 gauss 
#define LIS3MDL_SENSITIVITY_8G  3421         // LSB/gauss at 8 gauss 
#define LIS3MDL_SENSITIVITY_12G 2281         // LSB/gauss at 12 gauss 
#define LIS3MDL_SENSITIVITY_16G 1711         // LSB/gauss at 16 gauss 

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lis3mdl_data_t;

int lis3mdl_init(void);
int lis3mdl_read_mag(lis3mdl_data_t *data);
int lis3mdl_verify_device(void);
int lis3mdl_configure(void);

int lis3mdl_write_reg(uint8_t reg, uint8_t value);
int lis3mdl_read_multi_reg(uint8_t reg, uint8_t *data, size_t len);

#endif
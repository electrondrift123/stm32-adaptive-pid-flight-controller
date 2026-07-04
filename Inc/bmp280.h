#ifndef BMP280_H
#define BMP280_H

#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define BMP280_ADDRESS 0x76
#define AVG_WINDOW_SIZE 10 // was 10
// static float pressure_buffer[AVG_WINDOW_SIZE];
// static int buffer_index = 0;
// static bool buffer_filled = false;

typedef struct {
    float temperature;
    float pressure;
    float altitude;
} bmp280Data_t;

bool bmp280_init(I2C_HandleTypeDef *hi2c1);
bool bmp280_read(I2C_HandleTypeDef *hi2c1, bmp280Data_t *data);

void bmp280_reset_altitude_reference(void);

#endif // BMP280_H
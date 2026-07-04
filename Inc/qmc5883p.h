#ifndef QMC5883P_H
#define QMC5883P_H

#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define QMC5883P_ADDR 0x2C

static float minX = 1400, maxX = 2200; 
static float minY = 900, maxY = 1700; 
static float minZ = 5100, maxZ = 5500; 

typedef struct {
  float mx, my, mz;
  float angleYaw;
} magData_t;

// // Optional: Extract calibration separately
// typedef struct {
//   int16_t minX, maxX;
//   int16_t minY, maxY;
//   int16_t minZ, maxZ;
// } magCalib_t;

bool qmc5883p_init(I2C_HandleTypeDef *hi2c1);
bool qmc5883p_read(I2C_HandleTypeDef *hi2c1, magData_t *magData);  // still uses hardcoded min/max
void qmc5883p_updateYawWithTilt(magData_t* mag, float roll, float pitch);

void calibrate_compass(void);

#endif

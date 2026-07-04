#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include "i2c.h"
#include "math.h"
#include <stdint.h>
#include <stdbool.h>

#define MPU6050_ADDR 0x68
#define MPU6050_GYRO_SENSITIVITY 65.5f
#define MPU6050_ACCEL_SENSITIVITY 4096.0f
#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.0174533f

typedef struct {
 float ax, ay, az; // Accelerometer data
 float gx, gy, gz; // Gyroscope data
 float wx, wy, wz; // Angular velocity data
} mpu6050Data_t;

bool mpu6050_init(I2C_HandleTypeDef *hi2c1);
bool mpu6050_read(I2C_HandleTypeDef *hi2c1, mpu6050Data_t *d);

#endif // MPU6050_H
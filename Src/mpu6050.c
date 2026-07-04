#include "mpu6050.h"

bool mpu6050_init(I2C_HandleTypeDef *hi2c1){ // HAL version
 uint8_t data[2];

 // power management (0x6B -> 0x00)
 data[0] = 0x6B; // reg addr
 data[1] = 0x00; // value to write
 if (HAL_I2C_Master_Transmit(hi2c1, MPU6050_ADDR << 1 , data, 2, 100) != HAL_OK) return false; // always return to false!

 // LPF config
 data[0] = 0x1A;
 data[1] = 0x05; // 0x01 for faster response 
 if (HAL_I2C_Master_Transmit(hi2c1, MPU6050_ADDR << 1, data, 2, 100) != HAL_OK) return false;

 // Accel config (+/- 8g)
 data[0] = 0x1C;
 data[1] = 0x10;
 if (HAL_I2C_Master_Transmit(hi2c1, MPU6050_ADDR << 1, data, 2, 100) != HAL_OK) return false;

 // Gyro config (+/- 500 deg/s)
 data[0] = 0x1B;
 data[1] = 0x08;
 if (HAL_I2C_Master_Transmit(hi2c1, MPU6050_ADDR << 1, data, 2, 100) != HAL_OK) return false;

 // return true if all operations are okay:
 return true;
}

bool mpu6050_read(I2C_HandleTypeDef *hi2c1, mpu6050Data_t* d){
  // Read accelerometer
  uint8_t accel_reg_addr = 0x3B;
  uint8_t accel_data[6];

  // write
  if (HAL_I2C_Master_Transmit(hi2c1, MPU6050_ADDR << 1, &accel_reg_addr, 1, 100) != HAL_OK) return false;

  // read 6 bytes from the same device
  if (HAL_I2C_Master_Receive(hi2c1, MPU6050_ADDR << 1, accel_data, 6, 100) != HAL_OK) return false;

  int16_t ax = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  int16_t ay = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  int16_t az = (int16_t)((accel_data[4] << 8) | accel_data[5]);

  // Read gyroscope
  uint8_t gyro_reg_addr = 0x43;
  uint8_t gyro_data[6];

  if (HAL_I2C_Master_Transmit(hi2c1, MPU6050_ADDR << 1, &gyro_reg_addr, 1, 100) != HAL_OK) return false;
  if (HAL_I2C_Master_Receive(hi2c1, MPU6050_ADDR << 1, gyro_data, 6, 100) != HAL_OK) return false;

  int16_t gx = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
  int16_t gy = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
  int16_t gz = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

  d->gx = gx;
  d->gy = gy;
  d->gz = gz;
  d->ax = - ax / MPU6050_ACCEL_SENSITIVITY;
  d->ay = ay / MPU6050_ACCEL_SENSITIVITY;
  d->az = az / MPU6050_ACCEL_SENSITIVITY - 0.25f;

  d->wx   = (gx / MPU6050_GYRO_SENSITIVITY) * DEG_TO_RAD;
  d->wy = - (gy / MPU6050_GYRO_SENSITIVITY) * DEG_TO_RAD;
  d->wz = - (gz / MPU6050_GYRO_SENSITIVITY) * DEG_TO_RAD;

//   d->angleRoll = atan2f(d->ay, sqrtf(d->ax * d->ax + d->az * d->az)) * RAD_TO_DEG;
//   d->anglePitch = atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD_TO_DEG;

  return true;
}
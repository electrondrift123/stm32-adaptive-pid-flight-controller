/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "i2c.h"
#include "portmacro.h"
#include "task.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

#include "mpu6050.h"
#include "bmp280.h"
#include "qmc5883p.h"
#include "ema.h"
#include "madgwick_filter.h"
#include "kalman_altitude.h"
#include "sync.h"
#include "helper_functions.h"

// Global variables (to be transferred in: shared_data module)
mpu6050Data_t imuData;
bmp280Data_t baroData;
magData_t magData;

int16_t telemetry_data[5] = {0}; // [attitude(3), altitude(1), battery(1)]
int16_t cmd_data[5] = {0}; // [vz_cmd(1), roll(1), pitch(1), yaw(1), kill(1)]

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

// define freeRTOS tasks
void sensor_task(void *parameters);

BaseType_t result;
bool sensor_init_flag = false;
void MX_FREERTOS_Init(void) {
  result = xTaskCreate(
    sensor_task,
    "Sensor Task",
    256,
    NULL,
    1,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    sensor_init_flag = false;
    Error_Handler();
  }

  sensor_init_flag = true;
}

void sensor_task(void* parameters) {  // 1 kHz
  (void)parameters;

  const TickType_t intervalTicks = pdMS_TO_TICKS(1);  // 1ms = 1 kHz
  TickType_t lastWakeTime = xTaskGetTickCount();

  float local_altitude; 
  float ax, ay, az, wx, wy, wz, mx, my, mz; // Madgwick

  ema_t axLPF;
  ema_t ayLPF;
  ema_t azLPF;
  ema_t wxLPF;
  ema_t wyLPF;
  ema_t wzLPF;
  ema_t raw_alt_LPF;
  ema_t vzLPF;
  ema_t azTrueLPF;

  KalmanState_t kalmanState;


  // init for Accel LPF PT1
  emaInit(&axLPF, 1.0f, 3.0f, 1000.0f); // 15 Hz cutoff
  emaInit(&ayLPF, 1.0f, 3.0f, 1000.0f); // 15 Hz cutoff
  emaInit(&azLPF, 1.0f, 3.0f, 1000.0f); // 15 Hz cutoff

  emaInit(&wxLPF, 1.0f, 100.0f, 1000.0f); // 100 Hz cutoff
  emaInit(&wyLPF, 1.0f, 100.0f, 1000.0f); // 100 Hz cutoff
  emaInit(&wzLPF, 1.0f, 100.0f, 1000.0f); // 100 Hz cutoff

  emaInit(&raw_alt_LPF, 1.0f, 5.0f, 1000.0f); //was: PT1, fc = 1 Hz, fs = 1 kHz for raw altitude measurement smoothing
  emaInit(&vzLPF, 1.0f, 5.0f, 200.0f);

  float dt;

  kalman_init(&kalmanState); // Initialize Kalman filter state

  static int kalman_counter = 0; // run every 100 Hz
  const int kalman_interval = 5; //was: 10 ms = 100 Hz

  float fusedAlt = 0.0f;
  float velocity_z = 0.0f;
  float accel_bias = 0.0f;

  float az_true = 0.0f; // in g units
  float az_true_f = 0.0f;
  emaInit(&azTrueLPF, 1.0f, 5.0f, 1000.0f); // PT1 for vertical acceleration in world frame, fc = 2 Hz, fs = 1 kHz (for Kalman fusion)

  for (;;) {
    dt = intervalTicks * portTICK_PERIOD_MS / 1000.0f; // dt in seconds

    kalman_counter++;

    // read the BMP280 sensor
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      // read MPU6050, BMP280, Magnetometer sensor: 
      if (mpu6050_init(&hi2c1) && bmp280_init(&hi2c1) && qmc5883p_init(&hi2c1)) {

        // Successfully read MPU6050 data (acc_xyz is in g-units)
        ax = imuData.ax;
        ay = imuData.ay;
        az = imuData.az;
        wx = imuData.wx;
        wy = imuData.wy;
        wz = imuData.wz;
        mx = magData.mx;
        my = magData.my;
        mz = magData.mz;

        // Apply EMA filter to accelerometer data
        emaUpdate(&axLPF, ax);
        emaUpdate(&ayLPF, ay);
        emaUpdate(&azLPF, az);
        emaUpdate(&wxLPF, wx);
        emaUpdate(&wyLPF, wy);
        emaUpdate(&wzLPF, wz);

        ax = axLPF.output; // Get filtered X
        ay = ayLPF.output; // Get filtered Y
        az = azLPF.output; // Get filtered Z
        wx = wxLPF.output; // Get filtered X
        wy = wyLPF.output; // Get filtered Y
        wz = wzLPF.output; // Get filtered Z

        local_altitude = baroData.altitude; // Store altitude locally
       
      }else {
        // usart print: 
        Error_Handler();
      }
      xSemaphoreGive(i2cMutex);
    }

    // update the Madgwick's data:
    // if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE){
    //   MadgwickSensorList[0] = ax; // Accel X
    //   MadgwickSensorList[1] = ay; // Accel Y
    //   MadgwickSensorList[2] = az; // Accel Z
    //   MadgwickSensorList[3] = wx; // Gyro X
    //   MadgwickSensorList[4] = wy; // Gyro Y
    //   MadgwickSensorList[5] = wz; // Gyro Z
    //   MadgwickSensorList[6] = mx; // Mag X
    //   MadgwickSensorList[7] = my; // Mag Y
    //   MadgwickSensorList[8] = mz; // Mag Z

    //   az_true = getTrueAz(&madData, az);  // get vertical acceleration in g units

    //   xSemaphoreGive(madgwickMutex);
    // }

    // --- call LP filter ---
    emaUpdate(&raw_alt_LPF, local_altitude); // Update raw altitude LPF
    local_altitude = raw_alt_LPF.output; // Get smoothed altitude for fusion

    emaUpdate(&azTrueLPF, az_true); // LPF for vertical acceleration in world frame

    if (kalman_counter >= kalman_interval) { // 100 Hz
      az_true_f = azTrueLPF.output; // Get smoothed vertical acceleration in world frame for fusion
      if (fabsf(az_true_f - 1.0f) < 0.10f) {  // Ignore small accelerations
        az_true_f = 1.0f;
      }
      az_true_f = (az_true_f - 1.0f) * 9.81f; 

      az_true_f = constrainFloat(az_true_f, -20.0f, 20.0f); // Constrain to reasonable range for a drone (tune as needed)

      kalman_update(&kalmanState, local_altitude, az_true_f, dt * kalman_interval); // Kalman filter update for altitude estimation
      fusedAlt = kalmanState.S[0]; // Get altitude estimate from Kalman filter
      velocity_z = kalmanState.S[1]; // Get vertical velocity estimate from Kalman filter
      accel_bias = kalmanState.S[2]; // see if converge to -0.30

     
      emaUpdate(&vzLPF, velocity_z);
      velocity_z = vzLPF.output; // Get smoothed vertical velocity

      float vz_deadband = 0.15f; // was 20
      // if (fabs(az_true_f) < 0.1f && fabs(velocity_z) < 0.3f) vz_deadband = 0.05f;
      if (fabsf(velocity_z) < vz_deadband) velocity_z = 0.0f; // deadband to prevent jitter around zero velocity

      // leakage for vz
      const float vz_leak = 0.999f;
      velocity_z *= vz_leak;

      kalman_counter = 0; // reset counter
    }

    // Update shared data: BMP280 altitude
    // if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE){
    //   telemetry[0] = fusedAlt; // Update altitude in telemetry
    //   telemetry[1] = velocity_z; // Update vertical velocity in telemetry
    //   xSemaphoreGive(telemetryMutex);
    // }

    // debug printf
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
    //   // Serial.print("Euler: ");
    //   // Serial.print(madData.roll); Serial.print(", ");
    //   // Serial.print(madData.pitch); Serial.print(", ");
    //   // Serial.println(madData.yaw);

    //   // Serial.print("Vz: "); Serial.print(velocity_z); Serial.print(", ");
    //   // Serial.print("alt: "); Serial.print(fusedAlt); Serial.print(", bias: ");
    //   // Serial.println(accel_bias); 

    //   // Serial.print("az true f: "); Serial.println(az_true_f);

    //   // Serial.println(az_world); // 1g at ground, positive upward
    //   // Serial.print("raw alt: "); Serial.print(local_altitude); Serial.print(", az true: ");
    //   // Serial.println(((az_world - 1.0f) * 9.81));

    //   // Serial.print(mx); Serial.print(", "); Serial.print(my); Serial.print(", "); Serial.println(mz);
    //   xSemaphoreGive(serialMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, intervalTicks); 
  }
}
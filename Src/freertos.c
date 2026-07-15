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
#include "timers.h"
#include "i2c.h"
#include "portmacro.h"
#include "projdefs.h"
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
#include "indicators.h"
#include "lygapid.h"
#include "velocity_z_control.h"
#include "rx_com.h"

// DEFINE PRIORITY LEVELS (status: working)
#define PRIORITY_PID_FUSION       1 // 500 Hz -> try prio: 2 (radio task starve)
#define PRIORITY_WDT              1 // 1 Hz
#define PRIORITY_SENSOR_READ      1 // 1 kHz
#define PRIORITY_RADIO            1 // Interrupt driven
#define PRIORITY_BLINK            1 // 1 Hz
#define PRIORITY_BATTERY_MONITOR  1 // 10 Hz

// Global variables (to be transferred in: shared_data module)

// Failsafe for Radio          
TimerHandle_t linkWatchdogTimer = NULL;
const TickType_t LINK_TIMEOUT_MS = 200;           
volatile bool connection_ok = false;

mpu6050Data_t imuData;
bmp280Data_t baroData;
magData_t magData;
MadgwickData_t madData;

KalmanState_t kalmanState;

LyGAPIDControllerData_t pidRoll, pidPitch, pidRollRate, pidPitchRate, pidYawRate;

int16_t telemetry[5] = {0}; // [alt, vz, _, _, vbat]
int16_t cmd[5] = {0}; // [vz_cmd, yaw_rate_cmd, pitch_cmd, roll_cmd, kill]

float MadgwickSensorList[9] = {0}; // [a(3), w(3), m(3)]
float eulerAngles[3] = {0}; // roll, pitch, yaw angles

TaskHandle_t radioTaskHandle;

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

// define freeRTOS tasks
void blink_task(void *parameters);
void sensor_task(void *parameters);
void controller_task(void *parameters);
void rx_task(void *parameters);
void vbat_task(void *parameters);

// ------------------- ISR -------------------
void nrfInterruptHandler(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(radioTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void fastRadioRecoverRX(void) {
  radio_stopListening();
  radio_flush_rx();
  radio_flush_tx();
  radio_startListening();
  radio_clearStatusFlags();
}

// === Timer callback (runs in timer task context!) ===
static void linkTimeoutCallback(TimerHandle_t xTimer){
  (void) xTimer;

  connection_ok = false;

  // Safest & fastest way for very short write
  taskENTER_CRITICAL();
  cmd[0] = 0.0f; // zero Vcmd
  cmd[1] = 0.0f; // zero yaw input
  cmd[2] = 0.0f; // zero pitch input
  cmd[3] = 0.0f; // zero roll input
  if (cmd[4] == 0.0f) cmd[5] = 1.0f;   // set E-landing flag;   
  taskEXIT_CRITICAL();
}

void initLinkWatchdog(void){
  if (linkWatchdogTimer != NULL) {
    // already created → avoid double creation
    return;
  }

  linkWatchdogTimer = xTimerCreate(
    "LinkWD",
    LINK_TIMEOUT_MS / portTICK_PERIOD_MS, // timer period in ticks
    pdTRUE,                 // auto-reload
    NULL,
    linkTimeoutCallback
  );

  if (linkWatchdogTimer == NULL) {
      // Critical error - handle somehow (LED blink fast, Serial error, etc.)
      // For development you can leave it like this:
      // for(;;) { /* fail safe loop */ }
      connection_ok = false;
      taskENTER_CRITICAL();
      cmd[4] = 1.0f;  // stay killed
      taskEXIT_CRITICAL();
  }

  // We do NOT start it here — first valid packet will start/reset it
}




BaseType_t result;
bool init_flag = false; // Flag to indicate that FreeRTOS tasks have been initialized
void MX_FREERTOS_Init(void) {
  result = xTaskCreate(
    blink_task,
    "Blink Task",
    128,
    NULL,
    PRIORITY_BLINK,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    init_flag = false;
    buzz_error();
    Error_Handler();
  }

  result = xTaskCreate(
    sensor_task,
    "Sensor Task",
    256,
    NULL,
    PRIORITY_SENSOR_READ,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    init_flag = false;
    buzz_error();
    Error_Handler();
  }

  result = xTaskCreate( // pid controller & fusion
    controller_task,
    "PID & Fusion task",
    256,
    NULL,
    PRIORITY_PID_FUSION,
    NULL
  );
  if (result != pdPASS) {
    init_flag = false;
    buzz_error();
    Error_Handler();
  }

  result = xTaskCreate(
    rx_task,
    "nRF24 RX task",
    256,
    NULL,
    PRIORITY_RADIO,
    &radioTaskHandle
  );
  if (result != pdPASS) {
    init_flag = false;
    buzz_error();
    Error_Handler();
  }

  result = xTaskCreate(
    vbat_task,
    "Battery Monitor Task",
    128,
    NULL,
    PRIORITY_BATTERY_MONITOR,
    NULL
  );
  if (result != pdPASS) {
    init_flag = false;
    buzz_error();
    Error_Handler();
  }

  init_flag = true;
}

void blink_task(void *parameters){
  (void)parameters;
  const TickType_t intervalTicks = pdMS_TO_TICKS(500); // 500 ms = 1 Hz
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;){
    led_toggle();
    vTaskDelayUntil(&lastWakeTime, intervalTicks);
  }
}

void sensor_task(void* parameters) {  // 1 kHz
  (void)parameters;

  const TickType_t intervalTicks = pdMS_TO_TICKS(1);  // 1ms = 1 kHz
  TickType_t lastWakeTime = xTaskGetTickCount();

  float local_altitude; 
  float ax, ay, az, wx, wy, wz, mx, my, mz; // Madgwick

  ema_t axLPF, ayLPF, azLPF;
  ema_t wxLPF, wyLPF, wzLPF;
  ema_t raw_alt_LPF;
  ema_t vzLPF;
  ema_t azTrueLPF;

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
    if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      MadgwickSensorList[0] = ax; // Accel X
      MadgwickSensorList[1] = ay; // Accel Y
      MadgwickSensorList[2] = az; // Accel Z
      MadgwickSensorList[3] = wx; // Gyro X
      MadgwickSensorList[4] = wy; // Gyro Y
      MadgwickSensorList[5] = wz; // Gyro Z
      MadgwickSensorList[6] = mx; // Mag X
      MadgwickSensorList[7] = my; // Mag Y
      MadgwickSensorList[8] = mz; // Mag Z

      az_true = getTrueAz(&madData, az);  // get vertical acceleration in g units

      xSemaphoreGive(madgwickMutex);
    }

    // call LP filter
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
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      telemetry[0] = fusedAlt; // Update altitude in telemetry
      telemetry[1] = velocity_z; // Update vertical velocity in telemetry
      xSemaphoreGive(telemetryMutex);
    }

    // // debug printf (one task activated at a time)
    // if (xSemaphoreTake(uartMutex, portMAX_DELAY)){
    //   log_printf("Euler (roll, pitch, yaw): %.2f, %.2f, %.2f\r\n", madData.roll, madData.pitch, madData.yaw);

    //   xSemaphoreGive(uartMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, intervalTicks); 
  }
}

void controller_task(void *parameters){
  (void)parameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(2);  // 500 Hz
  const float dt = 0.002f;

  // ------ Initialize Adaptive PID (ATTi Mode) -------

  // outer loop: P-controller for roll, pitch, yaw angles (unit: rad)
  initLyGAPID(&pidRoll,  P, 0.0f, 0.0f, GAMMA_B, SIGMA, U_MAX_ROLL, CONTROLLER_MODE);
  initLyGAPID(&pidPitch, P, 0.0f, 0.0f, GAMMA_B, SIGMA, U_MAX_PITCH, CONTROLLER_MODE);

  // inner loop: PID for rates (unit: rad/sec)
  initLyGAPID(&pidRollRate,  KP, KI, KD, GAMMA_B, SIGMA, U_MAX_ROLL_RATE, CONTROLLER_MODE);
  initLyGAPID(&pidPitchRate, KP, KI, KD, B_SIGN, SIGMA, U_MAX_PITCH_RATE, CONTROLLER_MODE);
  initLyGAPID(&pidYawRate,   KP, KI, KD, B_SIGN, SIGMA, U_MAX_YAW_RATE, CONTROLLER_MODE); 

  // ====================== CONFIG ======================
  const float BASE_THROTTLE = 1000.0f;      // Fixed base in mixer
  const float MAX_TB        = 555.0f;       // Maximum hover component for takeoff only (was 53% too low, 58% too high), [54, 55]
  const float HOVER_TB      = 545.0f;       // Starting guess - tune this later

  const float KP_VZ = 143.0f; // was 140: [140, 180], 130 was too low
  const float KI_VZ = 0.5f; // was 0.5 good, [0.5, 2]
  const float VZ_OUTPUT_LIMIT = 200.0f; // already good

  const float ARM_HOLD_TIME = 2.1f; // 2.1 sec

  const float MOTOR_MIN = 1050.0f;
  const float MOTOR_MAX = 2000.0f;

  bool KILL_MOTORS = false;
  bool E_LAND = false; // Emergency Landing flag if signal is loss

  // ====================== VARIABLES ======================
  float vz_cmd = 0.0f;                    // Raw stick [-80,100]
  float vz_cmdFiltered = 0.0f;            // in m/s
  float velocity_z = 0.0f;
  float altitude = 0.0f;

  float roll, pitch, yaw;
  float rollRate, pitchRate, yawRate;

  float rollInput, pitchInput, yawInput;

  float rollInputFiltered = 0.0f;
  float pitchInputFiltered = 0.0f;
  float yawInputFiltered = 0.0f;

  // EMA filters
  ema_t R_LPF, P_LPF, Y_LPF, VZ_LPF;
  ema_t Vb_sag_comp_LPF;
  emaInit(&R_LPF, 1.0f, 15.0f, 500.0f);
  emaInit(&P_LPF, 1.0f, 15.0f, 500.0f);
  emaInit(&Y_LPF, 1.0f, 15.0f, 500.0f);
  emaInit(&VZ_LPF, 1.0f, 15.0f, 500.0f);

  emaInit(&Vb_sag_comp_LPF, 1.0f, 5.0f, 500.0f); // PT1 for voltage sag compensation

  // vertical controller mode
  typedef enum {
    VELOCITY_CONTROL,
    ALTITUDE_CONTROL
  } VerticalControlMode_t;

  VerticalControlMode_t vertical_mode = VELOCITY_CONTROL; // @ 100 Hz

  // State machine
  typedef enum {
    DISARMED,
    ARMED_IDLE,
    TAKEOFF,
    FLYING
  } FlightState_t;

  FlightState_t flightState = DISARMED;
  float armingTimer = 0.0f;
  float takeoffRamp = 0.0f;

  float vz_correction = 0.0f;
  float tb = 0.0f;                        // 0 → hover (this is your hover component)
  float Vb = 0.0; // 3s lipo: [10.8V, 12.6V]

  float motor_cmd[4] = {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};

  VelocityControlZData_t vz_in;
  init_vz_control(&vz_in, KP_VZ, KI_VZ, VZ_OUTPUT_LIMIT);

  float roll_rate_setpoint = 0.0f;
  float pitch_rate_setpoint = 0.0f;
  float yaw_rate_setpoint = 0.0f;

  float ax, ay, az, wx, wy, wz, mx, my, mz;

  int outer_loop_counter = 0;
  int print_counter = 0;

  vTaskDelay(pdMS_TO_TICKS(3000)); // 3 seconds wait for the motors to arm
 
  for (;;) {
    // Sensor Read (Madgwick + Telemetry)
    if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      ax = MadgwickSensorList[0]; ay = MadgwickSensorList[1]; az = MadgwickSensorList[2];
      wx = MadgwickSensorList[3]; wy = MadgwickSensorList[4]; wz = MadgwickSensorList[5];
      mx = MadgwickSensorList[6]; my = MadgwickSensorList[7]; mz = MadgwickSensorList[8];
      xSemaphoreGive(madgwickMutex);
    }

    MadgwickFilterUpdate(&madData, wx, wy, wz, ax, ay, az, mx, my, mz, dt);
    MadgwickGetEuler(&madData);

    if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      eulerAngles[0] = madData.roll;
      eulerAngles[1] = madData.pitch;
      eulerAngles[2] = madData.yaw;
      xSemaphoreGive(eulerAnglesMutex);
    }

    roll  = eulerAngles[0] * DEG_TO_RAD;
    pitch = eulerAngles[1] * DEG_TO_RAD;
    yaw   = eulerAngles[2] * DEG_TO_RAD;

    rollRate  = wx;
    pitchRate = wy;
    yawRate   = wz;

    // User Input 
    if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      vz_cmd     = cmd[0]; // [-0.8, 1.0] m/s
      yawInput   = cmd[1]; // [-180, 180] deg/s → rad/s
      pitchInput = cmd[2]; // [-20, 20] deg → rad
      rollInput  = cmd[3]; // [-20, 20] deg → rad

      KILL_MOTORS = (cmd[4] != 0.0f);
      // bool pilot_E_LAND = (cmd[5] != 0.0f);
      bool pilot_E_LAND = false; // temporarily true

      // Pilot E_LAND switch OR low battery triggers emergency descent
      // But KILL_MOTORS overrides everything
      if (!KILL_MOTORS) {
        if (pilot_E_LAND || (Vb < 10.9f)) E_LAND = true;
        else E_LAND = false;
      }
      
      xSemaphoreGive(nRF24Mutex);
    }

    // constrain redundancy 
    rollInput  = constrainFloat(rollInput, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    pitchInput = constrainFloat(pitchInput, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    yawInput   = constrainFloat(yawInput, -YAW_MAX, YAW_MAX);
    vz_cmd     = constrainFloat(vz_cmd, -0.8f, 1.0f);

    // Telemetry Update
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      altitude = telemetry[0];
      velocity_z = telemetry[1];
      Vb = telemetry[4] / 100.0f; // Convert to volts (3S LiPo) [10.8, 12.6] V

      xSemaphoreGive(telemetryMutex);
    }

    // E_LAND Behavior (execute in control loop, not in semaphore blocks)
    if (E_LAND && !KILL_MOTORS) {
      // Force descent unless pilot is actively climbing
      if (vz_cmd < 0.1f) {  // Not commanding significant climb
        vz_cmd = -0.4f;   // Safe descent rate, was 0.5
      }
      // If pilot gives positive vz_cmd, let them override
    }

    if (flightState == DISARMED || flightState == ARMED_IDLE) { 
      altitude = 0.0f;
      velocity_z = 0.0f;
      kalman_reset(&kalmanState);
    }

    if ((flightState == TAKEOFF || flightState == FLYING) && 
      (fabsf(roll) > MAX_SAFE_ANGLE_RAD || fabsf(pitch) > MAX_SAFE_ANGLE_RAD)) {
      KILL_MOTORS = true;
      flightState = DISARMED;
    }

    // ====================== ARMING (DJI Style) ======================
    bool sticksInArmPosition = (vz_cmd < -0.70f) && (fabsf(pitchInput) > 18.0f * DEG_TO_RAD) && (fabs(pitch) < 10.0f) && (fabs(roll) < 10.0f);

    if (flightState == DISARMED) {
      if (sticksInArmPosition) {
        armingTimer += dt;
        if (armingTimer >= ARM_HOLD_TIME) {
          flightState = ARMED_IDLE;
          armingTimer = 0.0f;
          tb = 0.0f;
          buzz_trigger(1, 120);                    // One short beep on arm
          for (int i = 0; i < 4; i++) motor_cmd[i] = 1150.0f;
        }
      } else {
        armingTimer = 0.0f;
      }
    }

    // ====================== STATE MACHINE ======================
    switch (flightState) {
      case DISARMED:
        tb = 0.0f;
        vz_in.is_flying = 0.0f;
        break;

      case ARMED_IDLE:
        tb = 0.0f;
        vz_in.is_flying = 0.0f;
        if (vz_cmd > -0.70f) { 
          flightState = TAKEOFF;
          takeoffRamp = 0.0f;
        }
        break;

      case TAKEOFF:
        vz_in.is_flying = 0.0f; // should i make this 0? let's try: 
        takeoffRamp += dt * 0.75f;           // ~1.3s ramp
        if (takeoffRamp > 1.0f) takeoffRamp = 1.0f;

        tb = takeoffRamp * MAX_TB;           // ramp 0 → 545

        if (takeoffRamp >= 1.0f && fabsf(velocity_z) < 0.5f) {
          flightState = FLYING;
        }
        break;

      case FLYING:
        vz_in.is_flying = 1.0f;
        // ====================== GROUND EFFECT COMPENSATION (E-LAND) ======================
        const float K_GE = 200.0f;  // Tune this (start 80-120)
        const float GE_THRESHOLD = 1.4f;
        const float R_PROP = 0.127f;  // 1045-inch prop radius
        float ge_comp = 0.0f;

        if (altitude < GE_THRESHOLD && E_LAND) {
          float z = altitude; // distance from ground in meters
          if (z < 0.3) z = 0.3; // prevent extreme compensation when very close to ground
          ge_comp = K_GE * (R_PROP / z) * (R_PROP / z);
        }

        // gain sched for voltage sag
        float Vb_gain = 50.0f;
        float drop = 12.6f - Vb; // how much voltage has dropped from ideal
        if (Vb < 12.0f && Vb > 11.4f) Vb_gain = 60.0f;
        else if (Vb < 11.4f) Vb_gain = 70.0f;

        float voltage_sag_com = Vb_gain * drop; // remove this in the future

        // voltage sag LPF
        // emaUpdate(&Vb_sag_comp_LPF, voltage_sag_com);
        // voltage_sag_com = Vb_sag_comp_LPF.output;

        tb = HOVER_TB + voltage_sag_com - ge_comp; // voltage sag compensation     

        break;
    }

    // ====================== FILTERING ======================
    emaUpdate(&R_LPF, rollInput);
    emaUpdate(&P_LPF, pitchInput);
    emaUpdate(&Y_LPF, yawInput);
    emaUpdate(&VZ_LPF, vz_cmd);

    rollInputFiltered  = constrainFloat(R_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    pitchInputFiltered = constrainFloat(P_LPF.output, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    yawInputFiltered   = constrainFloat(Y_LPF.output, -YAW_MAX, YAW_MAX);
    vz_cmdFiltered     = VZ_LPF.output;   // [-0.8,1.0] m/s

    // Attitude outer loop @ 100 Hz
    outer_loop_counter++;
    if (outer_loop_counter >= 5) {
      // Vertical Velocity Controller
      if ((flightState == TAKEOFF || flightState == FLYING) && vertical_mode == VELOCITY_CONTROL) {
        vz_correction = compute_vz_control(&vz_in, vz_cmdFiltered, velocity_z, dt);
      }
      roll_rate_setpoint  = computeLyGAPID_out(&pidRoll,  rollInputFiltered,  roll,  0.01f);
      pitch_rate_setpoint = computeLyGAPID_out(&pidPitch, pitchInputFiltered, pitch, 0.01f);
      outer_loop_counter = 0;
    }

    yaw_rate_setpoint = yawInputFiltered;

    bool isFlying = (flightState == FLYING);
    pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = isFlying ? 0.0f : 1.0f;

    float total_throttle = BASE_THROTTLE + tb + vz_correction;

    // Apply appropriate limits per state
    if (flightState == TAKEOFF || flightState == FLYING) {
      total_throttle = constrainFloat(total_throttle, 1450.0f, 1750.0f);
    }
    // DISARMED and ARMED_IDLE handle their own motor values directly

    // Motor Mixer
    if (KILL_MOTORS) {
      resetLyGAPID(&pidRoll); resetLyGAPID(&pidPitch);
      resetLyGAPID(&pidRollRate); resetLyGAPID(&pidPitchRate); resetLyGAPID(&pidYawRate);
      kalman_reset(&kalmanState);
      flightState = DISARMED;
      tb = 0.0f;
      vz_in.is_flying = 0.0f;
      pidRoll.landed = pidPitch.landed = pidRollRate.landed = pidPitchRate.landed = 1.0f;
      for (int i = 0; i < 4; i++) motor_cmd[i] = MOTOR_MIN;
    } else {
      float R_mix = constrainFloat(computeLyGAPID_in(&pidRollRate, roll_rate_setpoint, rollRate, dt), -U_MAX_ROLL_RATE, U_MAX_ROLL_RATE);
      float P_mix = constrainFloat(computeLyGAPID_in(&pidPitchRate, pitch_rate_setpoint, pitchRate, dt), -U_MAX_PITCH_RATE, U_MAX_PITCH_RATE);
      float Y_mix = constrainFloat(computeLyGAPID_yaw(&pidYawRate, yaw_rate_setpoint, yawRate, dt), -U_MAX_YAW_RATE, U_MAX_YAW_RATE);

      motor_cmd[0] = total_throttle + R_mix + P_mix - Y_mix;
      motor_cmd[1] = total_throttle - R_mix + P_mix + Y_mix;
      motor_cmd[2] = total_throttle + R_mix - P_mix + Y_mix;
      motor_cmd[3] = total_throttle - R_mix - P_mix - Y_mix;
    }

    for (int i = 0; i < 4; i++) {
      motor_cmd[i] = constrainFloat(motor_cmd[i], MOTOR_MIN, MOTOR_MAX);
    }

    // Motor Output
    TIM2->CCR1 = (uint16_t)motor_cmd[0];
    TIM2->CCR2 = (uint16_t)motor_cmd[1];
    TIM2->CCR3 = (uint16_t)motor_cmd[2];
    TIM2->CCR4 = (uint16_t)motor_cmd[3];

    // if (++print_counter >= 50) {
    //   print_counter = 0;
    // }

    // // logging (one task at a time for activation)
    // print_counter ++;
    // if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1)) == pdTRUE){
    //   print_couter = 0;
    //   log_printf("hey yo! \r\n");
    //   xSemaphoreGive(uartMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, interval);
  }
}

void rx_task(void *parameters) {
  (void)parameters;

  int16_t local_telemetry[6] = {0}; 
  int16_t rx_load[5] = {0};

  uint8_t PIPE_INDEX = 0x00;

  initLinkWatchdog();
  connection_ok = false;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint8_t flags = radio_clearStatusFlags();   

    if (flags & 0x40) { // 0x40 from the STATUS reg RX_DR bit
        while (radio_available()) {
            connection_ok = true;

            // Restart watchdog
            if (xTimerIsTimerActive(linkWatchdogTimer) == pdFALSE) {
              xTimerStart(linkWatchdogTimer, 0);
            } else {
              xTimerReset(linkWatchdogTimer, 0);
            }

            // Prepare ACK payload
            if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
              local_telemetry[0] = (int16_t)eulerAngles[0];
              local_telemetry[1] = (int16_t)eulerAngles[1];
              local_telemetry[2] = (int16_t)eulerAngles[2];
              xSemaphoreGive(eulerAnglesMutex);
            }
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
              local_telemetry[3] = (int16_t)(telemetry[0] * 100.0f);
              local_telemetry[4] = (int16_t)telemetry[4];
              local_telemetry[5] = (int16_t)(telemetry[1] * 100.0f);
              xSemaphoreGive(telemetryMutex);
            }

            radio_writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));

            radio_read(rx_load, sizeof(rx_load));

            // Process commands...
            float Tcmd = (float)rx_load[0] / 100.0f;
            float Ycmd = ((float)rx_load[1]) * DEG_TO_RAD;
            float Pcmd = ((float)rx_load[2] / 100.0f) * DEG_TO_RAD * (-1.0f);
            float Rcmd = ((float)rx_load[3] / 100.0f) * DEG_TO_RAD;
            float kill = (rx_load[4] == 0) ? 0.0f : 1.0f;

            if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                cmd[0] = constrainFloat(Tcmd, THROTTLE_MIN, THROTTLE_MAX);
                cmd[1] = constrainFloat(Ycmd, -YAW_MAX, YAW_MAX);
                cmd[2] = constrainFloat(Pcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
                cmd[3] = constrainFloat(Rcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
                cmd[4] = kill;
                xSemaphoreGive(nRF24Mutex);
            }
        }
    }

    // Handle TX FIFO issues (MAX_RT flag)
    if (flags & 0x10) {
      radio_flush_tx();
    }

    // === NEW: Periodic recovery check ===
    static uint32_t lastRecoverCheck = 0;
    if (xTaskGetTickCount() - lastRecoverCheck > pdMS_TO_TICKS(500)) { // was 200
        lastRecoverCheck = xTaskGetTickCount();
        
        if (!connection_ok) {
          fastRadioRecoverRX();     
        }
    }
  }
}


void vbat_task(void *parameters){
  (void)parameters;

  TickType_t interval = pdMS_TO_TICKS(20);
  TickType_t lastWakeTime = xTaskGetTickCount();

  float batteryVoltage; // Variable to hold battery voltage

  int counter = 0;
  float Vb_avg = 0.0f;

  ema_t VbLPF;
  emaInit(&VbLPF, 2.0f, 10.0f, 50.0f); // PT2, fc = 5 Hz, fs = 50 Hz (dt=0.02s) for battery voltage smoothing

  for (;;){
    // Read battery voltage
    counter++;
    uint16_t adc_value = readVbat();
    emaUpdate(&VbLPF, adc_value);
    adc_value = VbLPF.output; // Get filtered ADC value

    Vb_avg += (float)adc_value / 10.0f; // accumulate for averaging    

    if (counter == 10){
      batteryVoltage = Vb_avg * (12.6f / 2430.0f); // Convert ADC value to voltage (assuming 12.6V max and 12-bit ADC)

      // clamp battery level to reasonable range
      batteryVoltage = constrainFloat(batteryVoltage, 10.7f, 12.6f); // Constrain to reasonable range

      // Update shared data
      if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
        telemetry[4] = (int16_t)(100.0f * batteryVoltage); // Update battery voltage in telemetry (scaled by 100)
        xSemaphoreGive(telemetryMutex);
      }

      if (batteryVoltage < 10.8f){
        buzz_on();
        vTaskDelay(pdMS_TO_TICKS(10));
        buzz_off();
      }

      Vb_avg = 0.0f; // reset the voltage
      counter = 0; // reset the counter
    }
    
    vTaskDelayUntil(&lastWakeTime, interval); // Delay until the next cycle
  }
}
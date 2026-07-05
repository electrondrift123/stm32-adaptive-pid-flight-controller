/* 
Lyapunov-Guided Adaptive PID (LyGAPID) for Attitude Control

Ayob II Tuanadatu - Bachelor of Science in Electrical Engineering (2026)
*/

#ifndef LYGAPID_H
#define LYGAPID_H

#include <stdint.h>
#include "ema.h"
#include "helper_functions.h"

//////////////////// define macros
// outer loop (desired angular rate in rad/s): 180 deg/sec
#define U_MAX_ROLL          PI 
#define U_MAX_PITCH         PI 

// inner loop: it is pwm ticks in us for authority (PID 40% authority) 
#define U_MAX_ROLL_RATE     150.0f
#define U_MAX_PITCH_RATE    150.0f
#define U_MAX_YAW_RATE      100.0f

// Adaptive PID config: 
#define GAMMA_B         100.0f
#define SIGMA           0.0001f
#define B_SIGN          1.0f
#define CONTROLLER_MODE 1.0f // 0 = adaptive, 1 = static

// Initial P-PID Gains:
#define P   3.0f

#define KP  50.0f
#define KI  10.0f // was 15
#define KD  0.005f

// MAX commands in float: 
#define THROTTLE_MAX 100.0f // throttle is now [-0.8, 0.8] m/s cmd velocty z 
#define THROTTLE_MIN -100.0f
#define YAW_MAX             PI * 2.0f   // 360 deg/s max cmd
#define PITCH_ROLL_MAX      PI / 6.0f   // 30 deg max cmd

// failsafe (extreme tilt) -> kill motors
#define MAX_SAFE_ANGLE_RAD 75.0f * DEG_TO_RAD

/////////////////////// end - define macros

#define KP_OUT_MAX 8.0f // was 4.0f
#define KP_OUT_MIN 3.0f

#define KP_MAX 120.0f // was 60.0f
#define KP_MIN 40.0f

#define KI_MAX 30.0f
#define KI_MIN 0.0f // maybe try 10 later

#define KD_MAX 0.025f 
#define KD_MIN 0.005f

typedef struct {
    float Kp, Ki, Kd;

    float integral, prev_error, prev_derivative, i_limit;

    float gamma_base; // learning rate for adaptive PID
    float sigma; // relaxation factor for adaptive PID

    float output_limit;
    float sp;

    float mode; // 0 = adaptive, 1 = static
    float landed; // 1 = true

    ema_t pidLPF; // PT1 LPF filter for D-term

} LyGAPIDControllerData_t;

void initLyGAPID(LyGAPIDControllerData_t* lygapid, float kp, float ki, float kd,float gamma_base, float sigma, float output_limit, float mode);
     
float computeLyGAPID_out(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt); // outer loop (P controller)

float computeLyGAPID_in(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt); // inner loop (PID controller)

float computeLyGAPID_yaw(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt); // yaw inner loop (PI controller)

void resetLyGAPID(LyGAPIDControllerData_t* lygapid);

#endif // LYGAPID_H
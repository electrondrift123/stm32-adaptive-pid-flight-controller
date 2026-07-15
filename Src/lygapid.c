#include "lygapid.h"
#include "helper_functions.h" 
#include <stdbool.h>

void initLyGAPID(LyGAPIDControllerData_t* lygapid, float Kp, float Ki, float Kd, float gamma_base, float sigma, float output_limit, float mode){
    lygapid->Kp = Kp;
    lygapid->Ki = Ki;
    lygapid->Kd = Kd;

    lygapid->gamma_base = gamma_base;
    lygapid->sigma = sigma;

    lygapid->output_limit = output_limit;

    lygapid->mode = mode;

    lygapid->sp = 0.0f;
    lygapid->integral = 0.0f;
    lygapid->prev_error = 0.0f;
    lygapid->prev_derivative = 0.0f;

    lygapid->i_limit = lygapid->output_limit * 0.40f; // 40% of output limit

    lygapid->landed = 1.0f; // 1 = true, 0 = false

    emaInit(&lygapid->pidLPF, 2.0f, 50.0f, 500.0f); // fc > 40 Hz recommended
}

// for PITCH and ROLL positions
float computeLyGAPID_out(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt){
    float error = setpoint - actual;

    float u = lygapid->Kp * error;

    // anti-windup
    if (u > lygapid->output_limit){
        u = lygapid->output_limit;
    }
    else if (u < -lygapid->output_limit){
        u = -lygapid->output_limit;
    }

    lygapid->prev_error = error;

    if (lygapid->mode <= 0.0f){
        // Adaptation
        float gamma_p = 5.0f * lygapid->gamma_base;

        float dKp = gamma_p * error * error - lygapid->sigma * gamma_p * lygapid->Kp;

        lygapid->Kp += dKp * dt;

        if (lygapid->Kp > KP_OUT_MAX) lygapid->Kp = KP_OUT_MAX; // clamp
        else if (lygapid->Kp < KP_OUT_MIN) lygapid->Kp = KP_OUT_MIN;
    }
    else if (lygapid->mode == 1.0f){
        lygapid->Kp = KP_OUT_MIN; // fixed high Kp for output control
    }

    return u;
}

float computeLyGAPID_in(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt){
    float error = setpoint - actual;

    float derivative = (error - lygapid->prev_error) / dt;

    emaUpdate(&lygapid->pidLPF, derivative);
    derivative = lygapid->pidLPF.output;

    float u_ = lygapid->Kp * error + lygapid->Ki * lygapid->integral + lygapid->Kd * derivative;

    bool saturated = (u_ >= lygapid->output_limit || u_ <= -lygapid->output_limit);
    bool reducing_error = (error * u_ < 0);

    // integral update (Anti-windup)
    if (!saturated && (lygapid->landed < 1.0f)){ // not sat, not landed
        lygapid->integral += error * dt;
    }else if (lygapid->landed >= 1.0f) lygapid->integral = 0.0f; // reset integral when landed

    // integral clamping
    lygapid->integral = constrainFloat(lygapid->integral, -lygapid->i_limit, lygapid->i_limit);

    // final output
    float u = lygapid->Kp * error + lygapid->Ki * lygapid->integral + lygapid->Kd * derivative;

    // output clamping
    u = constrainFloat(u, -lygapid->output_limit, lygapid->output_limit);

    lygapid->prev_error = error;
    lygapid->prev_derivative = derivative;

    if (lygapid->mode <= 0.0f){
        // Adaptation
        float gamma_p = lygapid->gamma_base;
        float gamma_i = lygapid->gamma_base;
        float gamma_d = lygapid->gamma_base * 0.0001f;

        float dKp = gamma_p * error * error - lygapid->sigma * gamma_p * lygapid->Kp;
        float dKi = gamma_i * lygapid->integral * error - lygapid->sigma * gamma_i * lygapid->Ki;
        float dKd = gamma_d * derivative * error - lygapid->sigma * gamma_d * lygapid->Kd;

        lygapid->Kp += dKp * dt;
        lygapid->Ki += dKi * dt;
        lygapid->Kd += dKd * dt;

        // saturations
        lygapid->Kp = constrainFloat(lygapid->Kp, KP_MIN, KP_MAX);
        lygapid->Ki = constrainFloat(lygapid->Ki, KI_MIN, KI_MAX);
        lygapid->Kd = constrainFloat(lygapid->Kd, KD_MIN, KD_MAX);
    }
    else if (lygapid->mode == 1.0f){ // @ static PID
        lygapid->Kp = 25.0f; // was 25.0f
        lygapid->Ki = 5.0f; // was 10.0f 
        lygapid->Kd = 0.05f; // Can do 0.03 without motor heating. Was 0.005f. Next: [0.04, 0.05]
    }

    return u;
}

float computeLyGAPID_yaw(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt){
    float error = setpoint - actual;
    float u_ = lygapid->Kp * error + lygapid->Ki * lygapid->integral;

    bool saturated = (u_ >= lygapid->output_limit || u_ <= -lygapid->output_limit);
    bool reducing_error = (error * u_ < 0);

    // integral update (Anti-windup)
    if (!saturated && (lygapid->landed < 1.0f)){ // not sat, not landed
        lygapid->integral += error * dt;
    }else if (lygapid->landed >= 1.0f) lygapid->integral = 0.0f; // reset integral when landed

    // integral clamping
    lygapid->integral = constrainFloat(lygapid->integral, -lygapid->i_limit, lygapid->i_limit);

    // final output
    float u = lygapid->Kp * error + lygapid->Ki * lygapid->integral;

    // output clamping
    u = constrainFloat(u, -lygapid->output_limit, lygapid->output_limit);

    lygapid->prev_error = error;

    if (lygapid->mode <= 0.0f){
        // Adaptation
        float gamma_p = lygapid->gamma_base;
        float gamma_i = lygapid->gamma_base;

        float dKp = gamma_p * error * error - lygapid->sigma * gamma_p * lygapid->Kp;
        float dKi = gamma_i * lygapid->integral * error - lygapid->sigma * gamma_i * lygapid->Ki;

        lygapid->Kp += dKp * dt;
        lygapid->Ki += dKi * dt;

        // saturation
        lygapid->Kp = constrainFloat(lygapid->Kp, KP_MIN, KP_MAX); // clamp Kp
        lygapid->Ki = constrainFloat(lygapid->Ki, KI_MIN, KI_MAX); // clamp Ki
    }
    else if (lygapid->mode == 1.0f){
        lygapid->Kp = 25.0f; // fixed low Kp for yaw control (was 0.8f)
        lygapid->Ki = 10.0f; // fixed low Ki for yaw control
    }

    return u;
}


void resetLyGAPID(LyGAPIDControllerData_t* lygapid){
    lygapid->integral = 0.0f;
    lygapid->prev_error = 0.0f;
    lygapid->prev_derivative = 0.0f;
}
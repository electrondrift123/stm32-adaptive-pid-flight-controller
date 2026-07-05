#include "velocity_z_control.h"
#include "helper_functions.h"
#include "ema.h"

void init_vz_control(VelocityControlZData_t* vz, float kp, float ki, float output_limit) {
    vz->kp = kp;
    vz->ki = ki;
    vz->output_limit = output_limit;
    vz->integral = 0.0f;
    vz->is_flying = 0.0f;
}

float compute_vz_control(VelocityControlZData_t* vz, float v_cmd, float velocity_z, float dt) {
    
    if (vz->is_flying == 0.0f) {
        vz->integral = 0.0f;
        return 0.0f; 
    }

    // v_cmd & velocity_z are both in m/s 
    float error = v_cmd - velocity_z;       // positive = need more upward thrust

    float deadband = 0.05f; // 15 cm/s deadband
    if (fabs(error) < deadband) { // deadband to prevent jitter around zero velocity
        error = 0.0f;
    }

    // === Improved Integral with better anti-windup ===
    vz->integral += error * dt;

    // Optional: conditional integration (only integrate when not saturated)
    float u_temp = vz->kp * error + vz->ki * vz->integral;
    if (u_temp > vz->output_limit || u_temp < -vz->output_limit) {
        // don't integrate further in the direction that saturates
        vz->integral = vz->integral - error * dt;  // back off last step
    }

    // Hard clamp integral 
    const float integral_limit = 40.0f;
    vz->integral = constrainFloat(vz->integral, -integral_limit, integral_limit);

    // PI output
    float u = vz->kp * error + vz->ki * vz->integral;

    // Output saturation
    if (u > vz->output_limit)  u = vz->output_limit;
    if (u < -150.0f) u = - (150.0f); // was (-150)
    u = constrainFloat(u, -150.0f, vz->output_limit);

    return u;
}




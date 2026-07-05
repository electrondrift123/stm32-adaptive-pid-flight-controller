#ifndef VELOCITY_Z_CONTROL_H
#define VELOCITY_Z_CONTROL_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float output_limit;
    float altitude_sp;
    float integral;
    float is_flying; // 0 = grounded, 1 = flying 
} VelocityControlZData_t;

void init_vz_control(VelocityControlZData_t* vz_in, float kp, float ki, float output_limit);
float compute_vz_control(VelocityControlZData_t* vz_in, float v_cmd, float velocity_z, float dt);

#endif  // VELOCITY_CONTROL_Z_H
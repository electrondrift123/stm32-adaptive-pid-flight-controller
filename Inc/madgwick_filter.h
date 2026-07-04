#ifndef MADGWICK_H
#define MADGWICK_H

#include <math.h>
#include <stdint.h>

// System constants
// #define deltat 0.001f    // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f)  // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f)  // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError    // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift    // compute zeta

typedef struct {
    // Quaternion components
    float q0, q1, q2, q3;

    // Euler angles in degrees
    float roll;
    float pitch;
    float yaw;

    // sample rate
    float deltat;
} MadgwickData_t;


// Initialize the filter (optional default values)
// void Madgwick_init(MadgwickData_t* filterData, float sampleRateHz);

void Madgwick_resetState(void);
void Madgwick_initFromSensors(float ax, float ay, float az, float wx, float wy, float wz);

// Update the filter with new sensor data
void MadgwickFilterUpdate(MadgwickData_t* filterData,
                          float w_x, float w_y, float w_z,
                          float a_x, float a_y, float a_z,
                          float m_x, float m_y, float m_z,
                          float deltat);

// Convert quaternion to Euler angles (degrees)
void MadgwickGetEuler(MadgwickData_t* filterData);

// get the true az
float getTrueAz(MadgwickData_t* filterData, float a_z);

#endif // MADGWICK_H

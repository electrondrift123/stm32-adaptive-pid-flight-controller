#ifndef KALMAN_ALTITUDE_H
#define KALMAN_ALTITUDE_H

#include <stdint.h>
#include <math.h>
#include <string.h>

typedef struct {
    float S[3];        // State: [altitude, velocity, accel bias]
    float P[3][3];     // Covariance matrix (3x3)
} KalmanState_t;

void kalman_init(KalmanState_t* state);
void kalman_reset(KalmanState_t* state);
void kalman_update(KalmanState_t* state, float alt_raw, float az, float dt);

#endif
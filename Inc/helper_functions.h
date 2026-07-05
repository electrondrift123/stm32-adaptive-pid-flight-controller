#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define PI 3.1415f

float constrainFloat(float value, float low_limit, float up_limit);

void buzz_trigger(float count, float delay_ms);
void buzz_error(void);
void buzz_init_success(void);

uint16_t readVbat(void);

#endif
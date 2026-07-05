#ifndef EMA_H
#define EMA_H

#include <stdint.h>
#include <math.h>

typedef struct{
    float order; // [1,2]
    float fc; // cutoff frequency
    float fs; // sampling time
    float tau; // time constant
    float alpha; // [0,1]
    float x_prev;
    float x_prev2; // for 2nd order
    float output;
}ema_t;

void emaInit(ema_t* filter, float order, float fc, float fs);
void emaUpdate(ema_t* filter, float x);

#endif 
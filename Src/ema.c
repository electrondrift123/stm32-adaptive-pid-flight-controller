#include "ema.h"

void emaInit(ema_t* filter, float order, float fc, float fs){
    filter->order = order;
    if (filter->order == 2.0f){
        filter->fc = 1.55f * fc;
    }else{
        filter->fc = fc;
    }
    filter->fs = fs;
    filter->tau = 1.0f / (2.0f*3.1415f*filter->fc);
    filter->alpha = (1.0f / filter->fs) / (filter->tau + (1.0f / filter->fs));

    filter->x_prev = 0.0f;
    filter->x_prev2 = 0.0f; // for 2nd order
    filter->output = 0.0f;
}

void emaUpdate(ema_t* filter, float x) {
    if (filter->order == 1.0f) {
        filter->output = filter->alpha * x + (1.0f - filter->alpha) * filter->x_prev;
        filter->x_prev = filter->output;
    } else if (filter->order == 2.0f) {
        // Stage 1
        float stage1 = filter->alpha * x + (1.0f - filter->alpha) * filter->x_prev;
        filter->x_prev = stage1;
        // Stage 2
        filter->output = filter->alpha * stage1 + (1.0f - filter->alpha) * filter->x_prev2;
        filter->x_prev2 = filter->output;
    }
}

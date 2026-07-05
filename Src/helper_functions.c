#include "helper_functions.h"
#include "indicators.h"
#include "stm32f4xx_hal.h"

float constrainFloat(float value, float low_limit, float up_limit){
    if (value > up_limit) value = up_limit;
    else if (value < low_limit) value = low_limit;
    return value;
}

void buzz_trigger(float count, float delay_ms){
    for (int i = 0; i < (count); i++){
        buzz_toggle();
        HAL_Delay(delay_ms);
    }
}

void buzz_error(void){
    for (int i = 0; i < 10; i++){
        buzz_toggle();
        HAL_Delay(200);
    }
}

void buzz_init_success(void){
    // signal: --- --- (2 long buzzes)
    buzz_on();
    HAL_Delay(500);
    buzz_off();
    HAL_Delay(500);

    buzz_on();
    HAL_Delay(500);
    buzz_off();
    HAL_Delay(500);
}


uint16_t readVbat(void){
    //// TODO:
}
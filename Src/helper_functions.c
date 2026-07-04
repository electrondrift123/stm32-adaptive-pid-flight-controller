#include "helper_functions.h"

float constrainFloat(float value, float up_limit, float low_limit){
    if (value > up_limit) value = up_limit;
    else if (value < low_limit) value = low_limit;
    return value;
}
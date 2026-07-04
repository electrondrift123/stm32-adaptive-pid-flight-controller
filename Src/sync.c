#include "sync.h"

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t uartMutex;
SemaphoreHandle_t adcMutex;
SemaphoreHandle_t spiMutex;

bool sync_init(void) {
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) return false;
    uartMutex = xSemaphoreCreateMutex();
    if (uartMutex == NULL) return false;
    adcMutex = xSemaphoreCreateMutex();
    if (adcMutex == NULL) return false;
    spiMutex = xSemaphoreCreateMutex();
    if (spiMutex == NULL) return false;
    
    return true; // Return true if initialization is successful
}
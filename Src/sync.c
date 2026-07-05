#include "sync.h"
#include <string.h>

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t uartMutex;
SemaphoreHandle_t adcMutex;
SemaphoreHandle_t spiMutex;

SemaphoreHandle_t madgwickMutex;
SemaphoreHandle_t telemetryMutex;
SemaphoreHandle_t eulerAnglesMutex;
SemaphoreHandle_t nRF24Mutex;

bool sync_init(void) {
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) return false;
    uartMutex = xSemaphoreCreateMutex();
    if (uartMutex == NULL) return false;
    adcMutex = xSemaphoreCreateMutex();
    if (adcMutex == NULL) return false;
    spiMutex = xSemaphoreCreateMutex();
    if (spiMutex == NULL) return false;

    madgwickMutex = xSemaphoreCreateMutex();
    if (madgwickMutex == NULL) return false;
    telemetryMutex = xSemaphoreCreateMutex();
    if (telemetryMutex == NULL) return false;
    eulerAnglesMutex = xSemaphoreCreateMutex();
    if (eulerAnglesMutex == NULL) return false;
    nRF24Mutex = xSemaphoreCreateMutex();
    if (nRF24Mutex == NULL) return false;
    
    return true; // Return true if initialization is successful
}
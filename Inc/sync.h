#ifndef SYNC_H
#define SYNC_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>

// define global semaphores
extern SemaphoreHandle_t i2cMutex; // for sensors
extern SemaphoreHandle_t uartMutex; // for logging
extern SemaphoreHandle_t adcMutex; // for battery voltage reading
extern SemaphoreHandle_t spiMutex; // for radio communication

extern SemaphoreHandle_t madgwickMutex;
extern SemaphoreHandle_t telemetryMutex;
extern SemaphoreHandle_t eulerAnglesMutex;
extern SemaphoreHandle_t nRF24Mutex;

bool sync_init(void);

#endif // SYNC_H
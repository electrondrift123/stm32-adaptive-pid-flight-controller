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

bool sync_init(void);

#endif // SYNC_H
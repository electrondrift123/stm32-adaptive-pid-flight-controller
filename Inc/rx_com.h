#ifndef RX_COM_H
#define RX_COM_H

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
#include "gpio.h"

// CSN pin is PA4
// CE pin is PB13
// IRQ pin is PB1

bool rx_com_init(SPI_HandleTypeDef *hspi);

#endif // RX_COM_H
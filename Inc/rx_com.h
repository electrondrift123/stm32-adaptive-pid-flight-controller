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

bool radio_available(void); 
void radio_writeAckPayload(); // params: pipe index, payload (list), size
void radio_read(); // buffer (list), size(buffer)
void radio_startListening(void);
void radio_stopListening(void);
void radio_flush_tx(void); // Handle TX FIFO issues (MAX_RT flag)
void radio_flush_rx(void);
uint8_t radio_clearStatusFlags(void);

#endif // RX_COM_H
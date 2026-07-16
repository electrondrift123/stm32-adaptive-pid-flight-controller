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

bool radio_available(SPI_HandleTypeDef *hspi); 
void radio_startListening();
void radio_stopListening();
void radio_flush_tx(SPI_HandleTypeDef *hspi); // Handle TX FIFO issues (MAX_RT flag)
void radio_flush_rx(SPI_HandleTypeDef *hspi);
uint8_t radio_clearStatusFlags(SPI_HandleTypeDef *hspi);

void radio_writeAckPayload(SPI_HandleTypeDef *hspi, uint8_t pipe, const void* ack_buff, uint8_t len);
void radio_read(SPI_HandleTypeDef *hspi, void* rx_buff, uint8_t len);

#endif // RX_COM_H
#include "rx_com.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>

// define macros for the nRF24L01+ commands (table 16, p46 of the datasheet)
#define R_REGISTER       0x00     // 1-5 LSByte first (000A AAAA), A AAAA = register address
#define W_REGISTER       0x20     // 1-5 LSByte first (001A AAAA), A AAAA = register address

#define R_RX_PAYLOAD    (0b0110 << 4) | (0b0001) // 1-32 Bytes, LSByte first
#define W_TX_PAYLOAD    (0b1010 << 4) | (0b0001) // 1-32 Bytes, LSByte first
#define FLUSH_TX        (0b1110 << 4) | (0b0001)
#define FLUSH_RX        (0b1110 << 4) | (0b0010)
#define REUSE_TX_PL     (0b1110 << 4) | (0b0011)
#define ACTIVATE        (0b0101 << 4) | (0b0000)
#define R_RX_PL_WID     (0b0110 << 4) | (0b0000)
#define W_ACK_PAYLOAD   (0b1010 << 4) | (0b1000) // 1010 1PPP, LSByte first, PPP = pipe number [000 to 101]

// REG map (p53 of the datasheet)
#define CONFIG      0x00
#define PRIM_RX     (1 << 0)
#define PWR_UP      (1 << 1)
#define CRCO        (1 << 2) // 0 = 1 byte, 1 = 2 bytes
#define EN_CRC      (1 << 3)

#define EN_AA       0x01
#define ENAA_P0     (1 << 0) // enable auto ack in PIPE-0
#define ENAA_P1     (0 << 1)
#define ENAA_P2     (0 << 2)
#define ENAA_P3     (0 << 3)
#define ENAA_P4     (0 << 4)
#define ENAA_P5     (0 << 5)

#define EN_RXADDR   0x02
#define ERX_P0      (1 << 0) // enable data pipe 0
#define ERX_P1      (0 << 1)
#define ERX_P2      (0 << 2)
#define ERX_P3      (0 << 3)
#define ERX_P4      (0 << 4)
#define ERX_P5      (0 << 5)

#define SETUP_AW     0x03
#define AW_5BYTES    0b11 // 5 bytes address width

#define SETUP_RETR   0x04 // Not Applicable for RX mode, but still needs to be set for TX mode
#define ARC         0b0101 // Auto Retransmit Count (5 retries)
#define ARD         0b0100 // Auto Retransmit Delay ((4+1)*250 us)

#define RF_CH       0x05 // RF Channel 108, F0 = 2400 + 108 = 2508 MHz
#define RF_CH_      (uint8_t)108

#define RF_SETUP    0x06
#define LNA_HCURR   (1 << 0) // Setup LNA gain
#define RF_PWR      (0b11 << 1) // Setup RF output power (11 = 0 dBm)
#define RF_DR       (0 << 3) // Setup RF data rate (0 = 1 Mbps, 1 = 2 Mbps)

#define STATUS      0x07
#define TX_FULL     (1 << 0) // TX FIFO full flag (i dont know if this is correct or needed)
#define RX_P_NO     (0b111 << 1) // Data pipe number for the payload available for reading from RX FIFO
#define MAX_RT      (1 << 4) // Maximum number of TX retransmits interrupt
#define TX_DS       (1 << 5) // Data Sent TX FIFO interrupt
#define RX_DR       (1 << 6) // Data Ready RX FIFO interrupt

#define RX_PW_P0    0x11 
#define RX_PW_P0_   32 // Number of bytes in RX payload in data pipe 0

#define DYNPD       0x1C
#define DPL_0       (1 << 0) // Enable dynamic payload length for data pipe 0

#define FEATURE     0x1D
#define EN_DYN_ACK  (1 << 0) // Enable the W_TX_PAYLOAD_NOACK command
#define EN_ACK_PAY  (1 << 1) // Enable payload with ACK
#define EN_DPL      (1 << 2) // Enable dynamic payload length

void CSN_LOW(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void CSN_HIGH(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t nrf24_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg){
    // prepare the command
    uint8_t cmd = R_REGISTER | reg; 

    // prepare tx/rx buffers
    uint8_t tx_buffer[2] = {cmd, 0x00}; // Command byte followed by a dummy byte to read the register
    uint8_t rx_buffer[2] = {0};

    // CSN (PA4): HIGH to LOW transition to select the device
    // pull CSN low (PA4) to select the device
    CSN_LOW();

    // set the command byte to read the specified register
    HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, 100);

    // pull CSN high again (PA4) to deselect the device
    CSN_HIGH();

    return rx_buffer[1]; // Return the value read from the register
}

void nrf24_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value){
    uint8_t cmd = W_REGISTER | reg; // Prepare the command byte

    // prepare the buffers for tx and rx
    uint8_t tx_buffer[2] = {cmd, value}; // Command byte followed by the value to write
    uint8_t rx_buffer[2] = {0};
    
    // pull CSN low (PA4) to select the device
    CSN_LOW();

    HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, 100);

    CSN_HIGH(); // pull CSN high again (PA4) to deselect the device
}

bool radio_available(SPI_HandleTypeDef *hspi){
    /* Checks if TX is available.
    This function reads the STATUS register (0x07) and 
    examines the RX_P_NO field (bits 3, 2, and 1).
    */
    uint8_t status = nrf24_read_reg(hspi, STATUS);
    uint8_t pipe_no = (status >> 1) & 0b111;

    if (pipe_no == 0b111 || pipe_no == 0b110) return false;

    return true;
} 

uint8_t radio_clearStatusFlags(SPI_HandleTypeDef *hspi){
    uint8_t status = nrf24_read_reg(hspi, STATUS);

    // clear bit: 4-6 by writing "1"
    nrf24_write_reg(hspi, STATUS, status);

    return status;
}

void radio_flush_tx(SPI_HandleTypeDef *hspi){
    uint8_t cmd = FLUSH_TX;
    uint8_t rx_buffer = 0x00;

    CSN_LOW(); // pull CSN low
    HAL_SPI_TransmitReceive(hspi, &cmd, &rx_buffer, 1, 100); // spi cmd
    CSN_HIGH(); // pull CSN high 
} 

void radio_flush_rx(SPI_HandleTypeDef *hspi){
    uint8_t cmd = FLUSH_RX;
    uint8_t rx_buffer = 0x00;

    CSN_LOW(); // pull CSN low
    HAL_SPI_TransmitReceive(hspi, &cmd, &rx_buffer, 1, 100); // spi cmd
    CSN_HIGH(); // pull CSN high 
}

void radio_startListening(){
    /* Standby-1 to RX mode by writing PRIM-RX & CE to 1.
    And since the PRIM_RX is already 1 from the initialization, 
    we can just focus on the CE pin by pulling it HIGH */

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // CE pin HIGH to enter RX mode
}

void radio_stopListening(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // CE pin HIGH to enter RX mode
}

bool rx_com_init(SPI_HandleTypeDef *hspi){ 
    // power up:
    nrf24_write_reg(hspi, CONFIG, PRIM_RX | PWR_UP | EN_CRC | CRCO); 
    HAL_Delay(2); // wait for the device to power up (tPD2STBY = 150 us max with external clock)

    // Standby-1 (init conifgs)
    nrf24_write_reg(hspi, EN_AA, ENAA_P0);
    nrf24_write_reg(hspi, EN_RXADDR, ERX_P0);
    nrf24_write_reg(hspi, SETUP_AW, AW_5BYTES);

    // check for the device 
    uint8_t check = nrf24_read_reg(hspi, SETUP_AW);
    if (check != AW_5BYTES) return false;

    nrf24_write_reg(hspi, RF_CH, RF_CH_);
    nrf24_write_reg(hspi, RF_SETUP, LNA_HCURR | RF_PWR | RF_DR);
    nrf24_write_reg(hspi, STATUS, RX_DR | TX_DS | MAX_RT); // clear any pending interrupts
    // nrf24_write_reg(hspi, RX_PW_P0, RX_PW_P0_); // not used in dynamic payload mode

    nrf24_write_reg(hspi, ACTIVATE, 0x73); // activates: R_RX_PL_WID, W_ACK_PAYLOAD,  W_TX_PAYLOAD_NOACK
    nrf24_write_reg(hspi, DYNPD, DPL_0);
    nrf24_write_reg(hspi, FEATURE, EN_DPL | EN_ACK_PAY | EN_DYN_ACK);

    // PRIM_RX, CE need to set to HIGH to enter RX mode
    radio_startListening(); // maybe leave this so we can be more flexible for listening timing

    return true; // proceed with RX mode
}

void radio_writeAckPayload(SPI_HandleTypeDef *hspi, uint8_t pipe, const void* ack_buff, uint8_t len){ 
    uint8_t cmd = W_ACK_PAYLOAD | pipe;

    const uint8_t *byte_buff = (const uint8_t*)ack_buff; // declare const for looping
    uint8_t rx_buffer = 0; // dummy

    CSN_LOW();

    // send cmd byte
    HAL_SPI_TransmitReceive(hspi, &cmd, &rx_buffer, 1, 100);
    
    // send the payload bytes one by one
    for (uint8_t i = 0; i < len; i++){
        HAL_SPI_TransmitReceive(hspi, &byte_buff[i], &rx_buffer, 1, 100);
    }

    CSN_HIGH(); 
}

void radio_read(SPI_HandleTypeDef *hspi, void* rx_buff, uint8_t len){ 
    uint8_t cmd = R_RX_PAYLOAD;
    uint8_t rx_dummy = 0;
    
    uint8_t tx_buffer = 0;
    uint8_t *rx_buffer = (uint8_t*)rx_buff; 

    CSN_LOW();
    
    // send the cmd:
    HAL_SPI_TransmitReceive(hspi, &cmd, &rx_dummy, 1, 100);

    // read the RX FIFO one by one
    for (uint8_t i = 0; i < len; i++){
        HAL_SPI_TransmitReceive(hspi, &tx_buffer, &rx_buffer[i], 1, 100);
    }

    CSN_HIGH();
}

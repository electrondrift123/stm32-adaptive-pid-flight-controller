#include "logging.h"
#include "usart.h"  
#include "dma.h"    

static char log_buffer[256]; 

void log_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    int len = vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    
    va_end(args);

    // If the string is empty or too long, don't send it
    if (len <= 0 || len >= (int)sizeof(log_buffer)) {
        return;
    }

    // Send the whole string in ONE DMA transfer (Non-blocking)
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)log_buffer, len);
}
#include "indicators.h"
#include "gpio.h"

void buzz_on(void){
    BUZZER_PORT->BSRR = (1U << BUZZER_PIN);  // Set pin high
}

void buzz_off(void){
    BUZZER_PORT->BSRR = (1U << (BUZZER_PIN + 16));  // Reset pin low
}

void buzz_toggle(void){
    BUZZER_PORT->ODR ^= (1U << BUZZER_PIN);  // Toggle output
}

void led_on(void){
    LED_PORT->BSRR = (1U << LED_PIN);
}

void led_off(void){
    LED_PORT->BSRR = (1U << (LED_PIN + 16));
}

void led_toggle(void){
    LED_PORT->ODR ^= (1U << LED_PIN);
}
#ifndef INDICATORS_H
#define INDICATORS_H

// Pin definitions - CMSIS macros
#define BUZZER_PORT     GPIOB
#define BUZZER_PIN      12

#define LED_PORT       GPIOC
#define LED_PIN        13

void buzz_on(void);
void buzz_off(void);
void buzz_toggle(void);

void led_on(void);
void led_off(void);
void led_toggle(void);

#endif // BUZZER_H
// PWM LED device driver - controls LED brightness via analogWrite
#ifndef DD_LED_PWM_H
#define DD_LED_PWM_H

#include <stdint.h>

#define DD_LED_PWM_MAX_COUNT 2

void    ddLedPwmSetup(int id, int pin);
void    ddLedPwmWrite(int id, uint8_t duty);   // 0..255
uint8_t ddLedPwmGetDuty(int id);

#endif

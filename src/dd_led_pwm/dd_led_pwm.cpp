// PWM LED device driver - supports multiple PWM-driven LEDs indexed by ID.
// The target pin must be a PWM-capable output on the MCU.
#include <Arduino.h>
#include "dd_led_pwm.h"

static int     ledPwmPins[DD_LED_PWM_MAX_COUNT];
static uint8_t ledPwmDuty[DD_LED_PWM_MAX_COUNT];

void ddLedPwmSetup(int id, int pin)
{
    if (id < 0 || id >= DD_LED_PWM_MAX_COUNT)
        return;

    ledPwmPins[id] = pin;
    ledPwmDuty[id] = 0;

    pinMode(pin, OUTPUT);
    analogWrite(pin, 0);
}

void ddLedPwmWrite(int id, uint8_t duty)
{
    if (id < 0 || id >= DD_LED_PWM_MAX_COUNT)
        return;

    ledPwmDuty[id] = duty;
    analogWrite(ledPwmPins[id], duty);
}

uint8_t ddLedPwmGetDuty(int id)
{
    if (id < 0 || id >= DD_LED_PWM_MAX_COUNT)
        return 0;

    return ledPwmDuty[id];
}

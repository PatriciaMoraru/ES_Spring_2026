// LED device driver - supports multiple LEDs indexed by ID
#include <Arduino.h>
#include "dd_led.h"

static int ledPins[DD_LED_MAX_COUNT];
static bool ledStates[DD_LED_MAX_COUNT];

void ddLedSetup(int ledId, int pin)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return;
    }

    ledPins[ledId] = pin;
    ledStates[ledId] = false;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void ddLedOn(int ledId)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return;
    }

    ledStates[ledId] = true;
    digitalWrite(ledPins[ledId], HIGH);
}

void ddLedOff(int ledId)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return;
    }

    ledStates[ledId] = false;
    digitalWrite(ledPins[ledId], LOW);
}

void ddLedToggle(int ledId)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return;
    }

    if (ledStates[ledId])
    {
        ddLedOff(ledId);
    }
    else
    {
        ddLedOn(ledId);
    }
}

bool ddLedIsOn(int ledId)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return false;
    }

    return ledStates[ledId];
}

// LED device driver - supports multiple LEDs indexed by ID
#include <Arduino.h>
#include "dd_led.h"

// Arrays storing pin number and state for each LED
static int ledPins[DD_LED_MAX_COUNT];
static bool ledStates[DD_LED_MAX_COUNT];

// Register an LED: assign pin, set as output, start OFF
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

// Turn the specified LED on
void ddLedOn(int ledId)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return;
    }

    ledStates[ledId] = true;
    digitalWrite(ledPins[ledId], HIGH);
}

// Turn the specified LED off
void ddLedOff(int ledId)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return;
    }

    ledStates[ledId] = false;
    digitalWrite(ledPins[ledId], LOW);
}

// Toggle the specified LED
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

// Return true if the specified LED is currently on
bool ddLedIsOn(int ledId)
{
    if (ledId < 0 || ledId >= DD_LED_MAX_COUNT)
    {
        return false;
    }

    return ledStates[ledId];
}

// LED device driver implementation
#include <Arduino.h>
#include "dd_led.h"

static int DD_LED_PIN;         // Pin number assigned to the LED
static bool ledState = false;  // Current state of the LED

// Configure the LED pin and start with LED off
void ddLedSetup(int pin)
{
    DD_LED_PIN = pin;
    pinMode(DD_LED_PIN, OUTPUT);
    ddLedOff();
}

// Turn the LED on
void ddLedOn()
{
    ledState = true;
    digitalWrite(DD_LED_PIN, HIGH);
}

// Turn the LED off
void ddLedOff()
{
    ledState = false;
    digitalWrite(DD_LED_PIN, LOW);
}

// Toggle the LED between on and off
void ddLedToggle()
{
    if(ledState)
    {
        ddLedOff();
    }
    else
    {
        ddLedOn();
    }
}

// Return true if the LED is currently on
bool ddLedIsOn()
{
    return ledState;
}
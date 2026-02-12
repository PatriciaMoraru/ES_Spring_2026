#include <Arduino.h>
#include "dd_led.h"

static int DD_LED_PIN;
static bool ledState = false;

void ddLedSetup(int pin)
{
    DD_LED_PIN = pin;
    pinMode(DD_LED_PIN, OUTPUT);
    ddLedOff();
}

void ddLedOn()
{
    ledState = true;
    digitalWrite(DD_LED_PIN, HIGH);
}

void ddLedOff()
{
    ledState = false;
    digitalWrite(DD_LED_PIN, LOW);
}

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

bool ddLedIsOn()
{
    return ledState;
}
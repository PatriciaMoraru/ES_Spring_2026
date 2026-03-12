// LDR analog sensor device driver
// Reads a photoresistor via ADC in a voltage divider configuration.
// Higher analog value = more light (LDR to Vcc, pull-down to GND).
//
// Connection detection: takes multiple rapid reads and checks jitter.
// A connected sensor gives stable readings (jitter < 20).
// A floating pin oscillates wildly (jitter > 30).
#include <Arduino.h>
#include "dd_ldr.h"

#define STABILITY_SAMPLES 10
#define JITTER_THRESHOLD  20

static uint8_t ldrPin = A0;
static int lastRaw = 0;
static int lastJitter = 0;

void ddLdrSetup(uint8_t pin)
{
    ldrPin = pin;
    pinMode(pin, INPUT);
}

int ddLdrRead(void)
{
    int minVal = 1023;
    int maxVal = 0;

    for (int i = 0; i < STABILITY_SAMPLES; i++)
    {
        int val = analogRead(ldrPin);
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        lastRaw = val;
    }

    lastJitter = maxVal - minVal;
    return lastRaw;
}

uint8_t ddLdrGetPercent(void)
{
    return (uint8_t)map(lastRaw, 0, 1023, 0, 100);
}

int ddLdrGetJitter(void)
{
    return lastJitter;
}

bool ddLdrIsConnected(void)
{
    return (lastJitter < JITTER_THRESHOLD);
}

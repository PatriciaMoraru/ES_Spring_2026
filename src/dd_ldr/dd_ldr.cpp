// LDR analog sensor device driver
// Reads a photoresistor via ADC in a voltage divider configuration.
//
// Target module: KY-018 style board (fixed resistor between VCC and the
// signal pin, LDR between the signal pin and GND).  With this wiring the
// raw ADC value goes UP in the dark and DOWN in bright light, so the
// scaled light helpers (ddLdrGetLight / ddLdrGetPercent) invert the raw
// value and return 0 = dark, 100 = bright.  ddLdrRead() is left as the
// hardware-native raw value so diagnostics and jitter math stay honest.
//
// Connection detection: takes multiple rapid reads and checks jitter.
// A connected sensor gives stable readings (jitter < 20).
// A floating pin oscillates wildly (jitter > 30).
#include <Arduino.h>
#include "dd_ldr.h"

// Samples are spread across a window long enough to cover several PWM
// periods of a nearby LED driver (D9 PWM ~490 Hz => ~2 ms period).
// 16 samples * 700 us between reads => ~11 ms window (~5 full PWM cycles),
// which averages out the LED's on/off chopping and any 50/60 Hz mains
// hum. analogRead itself also takes ~100 us on AVR.
#define STABILITY_SAMPLES   16
#define SAMPLE_GAP_US      700
#define JITTER_THRESHOLD    20

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
    long sum   = 0;

    for (int i = 0; i < STABILITY_SAMPLES; i++)
    {
        int val = analogRead(ldrPin);
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        sum += val;

        if (i < STABILITY_SAMPLES - 1)
            delayMicroseconds(SAMPLE_GAP_US);
    }

    lastRaw    = (int)((sum + STABILITY_SAMPLES / 2) / STABILITY_SAMPLES);
    lastJitter = maxVal - minVal;
    return lastRaw;
}

float ddLdrGetLight(void)
{
    // Invert: high raw (dark) -> 0%, low raw (bright) -> 100%
    float pct = 100.0f - ((float)lastRaw * (100.0f / 1023.0f));
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

uint8_t ddLdrGetPercent(void)
{
    return (uint8_t)(ddLdrGetLight() + 0.5f);
}

int ddLdrGetJitter(void)
{
    return lastJitter;
}

bool ddLdrIsConnected(void)
{
    return (lastJitter < JITTER_THRESHOLD);
}

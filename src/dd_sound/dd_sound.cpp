// KY-037/KY-038 sound sensor device driver
// The analog output is centered around ~512 (Vcc/2) at silence.
// Sound causes deviations from center; peak-to-peak amplitude = loudness.
// We take rapid samples over a short window and compute peak-to-peak.
//
// Connection detection: a connected sound sensor has its DC average near 512.
// A floating pin has a random average (typically 200-400 range).
#include <Arduino.h>
#include "dd_sound.h"

#define SAMPLE_COUNT 50

static uint8_t soundPin = A1;
static int lastPeakToPeak = 0;
static int lastDcAvg = 0;

void ddSoundSetup(uint8_t pin)
{
    soundPin = pin;
    pinMode(pin, INPUT);
}

int ddSoundRead(void)
{
    int sampleMin = 1023;
    int sampleMax = 0;
    long sum = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        int val = analogRead(soundPin);

        if (val > sampleMax) sampleMax = val;
        if (val < sampleMin) sampleMin = val;
        sum += val;
    }

    lastPeakToPeak = sampleMax - sampleMin;
    lastDcAvg = (int)(sum / SAMPLE_COUNT);
    return lastPeakToPeak;
}

// Peak-to-peak rarely exceeds ~100 for even loud sounds.
// Map 0-80 raw to 0-100% so normal sounds register meaningfully.
#define SOUND_RAW_MAX 80

uint8_t ddSoundGetPercent(void)
{
    int clamped = constrain(lastPeakToPeak, 0, SOUND_RAW_MAX);
    return (uint8_t)map(clamped, 0, SOUND_RAW_MAX, 0, 100);
}

int ddSoundGetDcAvg(void)
{
    return lastDcAvg;
}

bool ddSoundIsConnected(void)
{
    // Sound sensor biases output at Vcc/2 (~512). Allow ±100 tolerance.
    return (lastDcAvg > 412 && lastDcAvg < 612);
}

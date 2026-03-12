// KY-037/KY-038 sound sensor device driver
// Measures ambient noise level via rapid ADC sampling (peak-to-peak).
#ifndef DD_SOUND_H
#define DD_SOUND_H

#include <stdint.h>

void    ddSoundSetup(uint8_t pin);
int     ddSoundRead(void);          // Returns peak-to-peak amplitude 0-1023
uint8_t ddSoundGetPercent(void);    // Returns sound level 0-100%
int     ddSoundGetDcAvg(void);      // DC average of last sample window (diagnostic)
bool    ddSoundIsConnected(void);   // True if DC offset is near Vcc/2 (~512)

#endif

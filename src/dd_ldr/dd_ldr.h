// LDR analog sensor device driver
//
// Hardware note: the supported module is the KY-018-style LDR board,
// which places the fixed resistor between VCC and the signal pin and the
// LDR between the signal pin and GND.  As a result, the raw ADC reads
// HIGH in the dark and LOW in bright light.  The percent/light helpers
// below invert that so callers always see:
//     0   = dark
//     100 = bright
#ifndef DD_LDR_H
#define DD_LDR_H

#include <stdint.h>

void    ddLdrSetup(uint8_t pin);
int     ddLdrRead(void);          // Returns raw ADC value 0-1023 (hardware-native, not inverted)
float   ddLdrGetLight(void);      // Returns light level 0.0-100.0 (0 = dark, 100 = bright)
uint8_t ddLdrGetPercent(void);    // Integer version of ddLdrGetLight (0 = dark, 100 = bright)
int     ddLdrGetJitter(void);     // Range of last multi-sample read (diagnostic)
bool    ddLdrIsConnected(void);   // True if readings are stable (not floating)

#endif

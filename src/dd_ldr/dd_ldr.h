// LDR analog sensor device driver
#ifndef DD_LDR_H
#define DD_LDR_H

#include <stdint.h>

void    ddLdrSetup(uint8_t pin);
int     ddLdrRead(void);          // Returns raw ADC value 0-1023
uint8_t ddLdrGetPercent(void);    // Returns light level 0-100%
int     ddLdrGetJitter(void);     // Range of last multi-sample read (diagnostic)
bool    ddLdrIsConnected(void);   // True if readings are stable (not floating)

#endif

// DHT11 digital temperature & humidity sensor device driver
#ifndef DD_DHT_H
#define DD_DHT_H

#include <stdint.h>

void  ddDhtSetup(uint8_t pin);
bool  ddDhtRead(void);            // Trigger a sensor read; returns true on success
float ddDhtGetTemperature(void);  // Last valid temperature in Celsius
float ddDhtGetHumidity(void);     // Last valid relative humidity in %
bool  ddDhtIsValid(void);         // Whether the last read succeeded

#endif

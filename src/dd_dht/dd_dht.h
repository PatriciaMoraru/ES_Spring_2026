// DHT digital temperature & humidity sensor device driver
#ifndef DD_DHT_H
#define DD_DHT_H

#include <stdint.h>

typedef enum { DD_DHT11, DD_DHT22 } DhtModel_t;

void  ddDhtSetup(uint8_t pin, DhtModel_t model);
bool  ddDhtRead(void);
float ddDhtGetTemperature(void);
float ddDhtGetHumidity(void);
bool  ddDhtIsValid(void);

#endif

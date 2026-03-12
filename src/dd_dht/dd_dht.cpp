// DHT11 digital temperature & humidity sensor device driver
// Wraps the markruys/DHT library. DHT11 supports reads every ~1-2 seconds.
#include <Arduino.h>
#include <DHT.h>
#include "dd_dht.h"

static DHT dht;
static float lastTemp  = 0.0;
static float lastHum   = 0.0;
static bool  lastValid = false;

void ddDhtSetup(uint8_t pin)
{
    dht.setup(pin, DHT::DHT11);
}

bool ddDhtRead(void)
{
    float temp = dht.getTemperature();
    float hum  = dht.getHumidity();

    if (isnan(temp) || isnan(hum) || dht.getStatus() != DHT::ERROR_NONE)
    {
        lastValid = false;
        return false;
    }

    lastTemp  = temp;
    lastHum   = hum;
    lastValid = true;
    return true;
}

float ddDhtGetTemperature(void)
{
    return lastTemp;
}

float ddDhtGetHumidity(void)
{
    return lastHum;
}

bool ddDhtIsValid(void)
{
    return lastValid;
}

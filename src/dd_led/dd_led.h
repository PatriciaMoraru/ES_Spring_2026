// LED device driver - controls multiple LEDs by ID
#ifndef DD_LED_H
#define DD_LED_H

#define DD_LED_MAX_COUNT 4

void ddLedSetup(int ledId, int pin);
void ddLedOn(int ledId);
void ddLedOff(int ledId);
void ddLedToggle(int ledId);
bool ddLedIsOn(int ledId);

#endif

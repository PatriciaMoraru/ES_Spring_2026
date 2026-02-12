// LED device driver - controls a single LED
#ifndef DD_LED_H
#define DD_LED_H

void ddLedSetup(int pin);  // Initialize the LED on the given pin
void ddLedOn();             // Turn LED on
void ddLedOff();            // Turn LED off
void ddLedToggle();         // Toggle LED state
bool ddLedIsOn();           // Get current LED state

#endif

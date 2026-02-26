// Heartbeat service - toggles an LED to show the scheduler is running
#include <Arduino.h>
#include "srv_heartbeat.h"
#include "dd_led/dd_led.h"

static int heartbeatLedId = 0;

void srvHeartbeatSetup(int ledId, int pin)
{
    heartbeatLedId = ledId;
    ddLedSetup(ledId, pin);
}

void srvHeartbeatTask(void)
{
    ddLedToggle(heartbeatLedId);
}

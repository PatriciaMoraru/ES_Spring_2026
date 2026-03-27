// Binary actuator device driver
#include <Arduino.h>
#include "dd_actuator.h"

static int _pins[DD_ACTUATOR_MAX_COUNT];
static bool _states[DD_ACTUATOR_MAX_COUNT];    // logical state: true = ON
static bool _activeLow[DD_ACTUATOR_MAX_COUNT]; // true = relay-style active-LOW output

static void _write(int id, bool logicalOn)
{
    // For active-LOW: ON  → LOW,  OFF → HIGH
    // For active-HIGH: ON → HIGH, OFF → LOW
    bool pinLevel = _activeLow[id] ? !logicalOn : logicalOn;
    digitalWrite(_pins[id], pinLevel ? HIGH : LOW);
}

void ddActuatorSetup(int id, int pin, bool activeLow)
{
    if (id < 0 || id >= DD_ACTUATOR_MAX_COUNT)
        return;
    _pins[id] = pin;
    _states[id] = false;
    _activeLow[id] = activeLow;

    digitalWrite(pin, activeLow ? HIGH : LOW);
    pinMode(pin, OUTPUT);
}

void ddActuatorOn(int id)
{
    if (id < 0 || id >= DD_ACTUATOR_MAX_COUNT)
        return;
    _states[id] = true;
    _write(id, true);
}

void ddActuatorOff(int id)
{
    if (id < 0 || id >= DD_ACTUATOR_MAX_COUNT)
        return;
    _states[id] = false;
    _write(id, false);
}

void ddActuatorToggle(int id)
{
    if (id < 0 || id >= DD_ACTUATOR_MAX_COUNT)
        return;
    if (_states[id])
        ddActuatorOff(id);
    else
        ddActuatorOn(id);
}

bool ddActuatorGetState(int id)
{
    if (id < 0 || id >= DD_ACTUATOR_MAX_COUNT)
        return false;
    return _states[id];
}

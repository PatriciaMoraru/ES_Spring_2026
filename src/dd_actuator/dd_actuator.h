// Binary actuator device driver — controls a single binary output (relay, LED)
#ifndef DD_ACTUATOR_H
#define DD_ACTUATOR_H

#include <stdbool.h>

#define DD_ACTUATOR_MAX_COUNT 4

void ddActuatorSetup(int id, int pin, bool activeLow);
void ddActuatorOn(int id);
void ddActuatorOff(int id);
void ddActuatorToggle(int id);
bool ddActuatorGetState(int id);

#endif

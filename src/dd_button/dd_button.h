// Button device driver - debounced button input by ID
#ifndef DD_BUTTON_H
#define DD_BUTTON_H

#define DD_BUTTON_MAX_COUNT 4
#define DD_BUTTON_DEBOUNCE_MS 20

void ddButtonSetup(int btnId, int pin);
bool ddButtonIsPressed(int btnId);
bool ddButtonPressed(int btnId);
bool ddButtonReleased(int btnId);
void ddButtonUpdate(int btnId);

#endif

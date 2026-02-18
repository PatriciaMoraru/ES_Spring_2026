// Keypad 4x4 device driver
#ifndef DD_KEYPAD_H
#define DD_KEYPAD_H

#define DD_KEYPAD_ROWS 4
#define DD_KEYPAD_COLS 4

void ddKeypadSetup();
char ddKeypadGetKey();
char ddKeypadWaitForKey();

#endif

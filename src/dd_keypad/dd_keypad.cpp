// Keypad 4x4 device driver - wraps the Keypad library
#include <Arduino.h>
#include <Keypad.h>
#include "dd_keypad.h"

#define KEYPAD_ROW_PIN_1 9
#define KEYPAD_ROW_PIN_2 8
#define KEYPAD_ROW_PIN_3 7
#define KEYPAD_ROW_PIN_4 6

#define KEYPAD_COL_PIN_1 5
#define KEYPAD_COL_PIN_2 4
#define KEYPAD_COL_PIN_3 3
#define KEYPAD_COL_PIN_4 2

static byte rowPins[DD_KEYPAD_ROWS] = {
    KEYPAD_ROW_PIN_1,
    KEYPAD_ROW_PIN_2,
    KEYPAD_ROW_PIN_3,
    KEYPAD_ROW_PIN_4
};

static byte colPins[DD_KEYPAD_COLS] = {
    KEYPAD_COL_PIN_1,
    KEYPAD_COL_PIN_2,
    KEYPAD_COL_PIN_3,
    KEYPAD_COL_PIN_4
};

static char keyMap[DD_KEYPAD_ROWS][DD_KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static Keypad keypad = Keypad(
    makeKeymap(keyMap),
    rowPins,
    colPins,
    DD_KEYPAD_ROWS,
    DD_KEYPAD_COLS
);

void ddKeypadSetup()
{
    // Keypad library handles pin configuration internally
}

char ddKeypadGetKey()
{
    return keypad.getKey();
}

char ddKeypadWaitForKey()
{
    return keypad.waitForKey();
}

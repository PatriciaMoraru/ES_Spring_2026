// Keypad 4x4 device driver - wraps the Keypad library
#include <Arduino.h>
#include <Keypad.h>
#include "dd_keypad.h"

// Row pin assignments (D9-D6)
#define KEYPAD_ROW_PIN_1 9
#define KEYPAD_ROW_PIN_2 8
#define KEYPAD_ROW_PIN_3 7
#define KEYPAD_ROW_PIN_4 6

// Column pin assignments (D5-D2)
#define KEYPAD_COL_PIN_1 5
#define KEYPAD_COL_PIN_2 4
#define KEYPAD_COL_PIN_3 3
#define KEYPAD_COL_PIN_4 2

// Pin arrays for Keypad library
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

// Standard 4x4 keypad layout
static char keyMap[DD_KEYPAD_ROWS][DD_KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// Create Keypad instance with the key map and pin arrays
static Keypad keypad = Keypad(
    makeKeymap(keyMap),
    rowPins,
    colPins,
    DD_KEYPAD_ROWS,
    DD_KEYPAD_COLS
);

// Keypad library configures pins internally during construction
void ddKeypadSetup()
{
}

// Non-blocking: returns pressed key or '\0' if none
char ddKeypadGetKey()
{
    return keypad.getKey();
}

// Blocking: waits until a key is pressed and returns it
char ddKeypadWaitForKey()
{
    return keypad.waitForKey();
}

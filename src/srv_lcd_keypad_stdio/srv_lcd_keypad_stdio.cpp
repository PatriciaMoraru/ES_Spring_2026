// LCD + Keypad STDIO service - redirects stdout to LCD, stdin to keypad
#include "srv_lcd_keypad_stdio.h"
#include <stdio.h>
#include <Arduino.h>

#include "dd_lcd/dd_lcd.h"
#include "dd_keypad/dd_keypad.h"

// '#' key acts as Enter/confirm
#define KEY_CONFIRM '#'

// FILE stream storage for STDIO redirection
static FILE* srvLcdKeypadStream;
static FILE srvLcdKeypadStreamStorage;

// Set up STDIO: redirect stdout to LCD, stdin to keypad
void srvLcdKeypadStdioSetup()
{
    // Register put/get callbacks for read-write mode
    fdev_setup_stream(&srvLcdKeypadStreamStorage,
                      srvLcdKeypadPutChar,
                      srvLcdKeypadGetChar,
                      _FDEV_SETUP_RW);

    srvLcdKeypadStream = &srvLcdKeypadStreamStorage;

    // Assign stream to standard output and input
    stdout = srvLcdKeypadStream;
    stdin = srvLcdKeypadStream;
}

// STDIO putchar callback: sends character to LCD
int srvLcdKeypadPutChar(char c, FILE* stream)
{
    ddLcdPrintChar(c);
    return 0;
}

// STDIO getchar callback: reads key from keypad (blocking)
// Maps '#' to newline so it acts as Enter
int srvLcdKeypadGetChar(FILE* stream)
{
    char key = ddKeypadWaitForKey();

    if (key == KEY_CONFIRM)
    {
        return '\n';
    }

    return (int)key;
}

// LCD + Keypad STDIO service - redirects stdout to LCD, stdin to keypad
#include "srv_lcd_keypad_stdio.h"
#include <stdio.h>
#include <Arduino.h>

#include "dd_lcd/dd_lcd.h"
#include "dd_keypad/dd_keypad.h"

#define KEY_CONFIRM '#'

static FILE* srvLcdKeypadStream;
static FILE srvLcdKeypadStreamStorage;

void srvLcdKeypadStdioSetup()
{
    fdev_setup_stream(&srvLcdKeypadStreamStorage,
                      srvLcdKeypadPutChar,
                      srvLcdKeypadGetChar,
                      _FDEV_SETUP_RW);

    srvLcdKeypadStream = &srvLcdKeypadStreamStorage;

    stdout = srvLcdKeypadStream;
    stdin = srvLcdKeypadStream;
}

int srvLcdKeypadPutChar(char c, FILE* stream)
{
    ddLcdPrintChar(c);
    return 0;
}

int srvLcdKeypadGetChar(FILE* stream)
{
    char key = ddKeypadWaitForKey();

    if (key == KEY_CONFIRM)
    {
        return '\n';
    }

    return (int)key;
}

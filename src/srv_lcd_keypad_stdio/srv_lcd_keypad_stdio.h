// LCD + Keypad STDIO service - redirects printf to LCD, getchar to keypad
#ifndef SRV_LCD_KEYPAD_STDIO_H
#define SRV_LCD_KEYPAD_STDIO_H

#include <stdio.h>

void srvLcdKeypadStdioSetup();
int srvLcdKeypadPutChar(char c, FILE* stream);
int srvLcdKeypadGetChar(FILE* stream);

#endif

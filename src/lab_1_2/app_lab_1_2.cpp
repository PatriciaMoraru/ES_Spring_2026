// Lab 1.2 - Password set-and-verify via LCD + Keypad using STDIO
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_1_2.h"
#include "dd_led/dd_led.h"
#include "dd_lcd/dd_lcd.h"
#include "dd_keypad/dd_keypad.h"
#include "srv_lcd_keypad_stdio/srv_lcd_keypad_stdio.h"

#define PIN_GREEN_LED    10
#define PIN_RED_LED      11

#define LED_GREEN        0
#define LED_RED          1

#define LCD_I2C_ADDRESS  0x27
#define LCD_COLS         16
#define LCD_ROWS         2

#define MAX_PIN_LENGTH   8
#define FEEDBACK_DELAY_MS 3000

#define KEY_BACKSPACE    '*'

static char pinSet[MAX_PIN_LENGTH + 1];
static char pinConfirm[MAX_PIN_LENGTH + 1];

static int readPin(char* buffer)
{
    int idx = 0;

    while (idx < MAX_PIN_LENGTH)
    {
        int c = getchar();

        if (c == '\n')
        {
            break;
        }

        if (c == KEY_BACKSPACE && idx > 0)
        {
            idx--;
            ddLcdSetCursor(idx, 1);
            putchar(' ');
            ddLcdSetCursor(idx, 1);
            continue;
        }

        if (c >= '0' && c <= '9')
        {
            buffer[idx] = (char)c;
            idx++;
            putchar('*');
        }
    }

    buffer[idx] = '\0';
    return idx;
}

void appLab12Setup()
{
    ddLedSetup(LED_GREEN, PIN_GREEN_LED);
    ddLedSetup(LED_RED, PIN_RED_LED);
    ddLcdSetup(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);
    ddKeypadSetup();
    srvLcdKeypadStdioSetup();
}

void appLab12Loop()
{
    ddLedOff(LED_GREEN);
    ddLedOff(LED_RED);

    ddLcdClear();
    printf("Set PIN:");
    ddLcdSetCursor(0, 1);
    int setLen = readPin(pinSet);

    if (setLen == 0)
    {
        ddLcdClear();
        printf("PIN empty!");
        delay(FEEDBACK_DELAY_MS);
        return;
    }

    ddLcdClear();
    printf("Confirm PIN:");
    ddLcdSetCursor(0, 1);
    readPin(pinConfirm);

    ddLcdClear();

    if (strcmp(pinSet, pinConfirm) == 0)
    {
        printf("Access Granted");
        ddLedOn(LED_GREEN);
    }
    else
    {
        printf("Access Denied");
        ddLedOn(LED_RED);
    }

    delay(FEEDBACK_DELAY_MS);
}

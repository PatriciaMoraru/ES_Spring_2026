// Lab 1.2 - Password set-and-verify via LCD + Keypad using STDIO
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_1_2.h"
#include "dd_led/dd_led.h"
#include "dd_lcd/dd_lcd.h"
#include "dd_keypad/dd_keypad.h"
#include "srv_lcd_keypad_stdio/srv_lcd_keypad_stdio.h"

// Hardware pin assignments
#define PIN_GREEN_LED    10
#define PIN_RED_LED      11

// LED IDs for dd_led module
#define LED_GREEN        0
#define LED_RED          1

// LCD configuration
#define LCD_I2C_ADDRESS  0x27
#define LCD_COLS         16
#define LCD_ROWS         2

// PIN input limits
#define MAX_PIN_LENGTH   8
#define FEEDBACK_DELAY_MS 3000

// Keypad special keys
#define KEY_BACKSPACE    '*'

// Buffers for set and confirm PIN entries
static char pinSet[MAX_PIN_LENGTH + 1];
static char pinConfirm[MAX_PIN_LENGTH + 1];

// Reads a PIN from the keypad, masking each digit with '*' on the LCD
// Returns the number of digits entered
static int readPin(char* buffer)
{
    int idx = 0;

    while (idx < MAX_PIN_LENGTH)
    {
        // Read one key via STDIO (blocks until key pressed)
        int c = getchar();

        // '#' mapped to '\n' by STDIO service = confirm
        if (c == '\n')
        {
            break;
        }

        // Handle backspace: erase last character on LCD
        if (c == KEY_BACKSPACE && idx > 0)
        {
            idx--;
            ddLcdSetCursor(idx, 1);
            putchar(' ');
            ddLcdSetCursor(idx, 1);
            continue;
        }

        // Accept only digits 0-9, ignore A-D
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

// Initialize all peripherals and STDIO
void appLab12Setup()
{
    ddLedSetup(LED_GREEN, PIN_GREEN_LED);
    ddLedSetup(LED_RED, PIN_RED_LED);
    ddLcdSetup(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);
    ddKeypadSetup();
    srvLcdKeypadStdioSetup();
}

// Main loop: set PIN, confirm PIN, show result
void appLab12Loop()
{
    // Reset LEDs
    ddLedOff(LED_GREEN);
    ddLedOff(LED_RED);

    // Step 1: Ask user to set a PIN
    ddLcdClear();
    printf("Set PIN:");
    ddLcdSetCursor(0, 1);
    int setLen = readPin(pinSet);

    // Reject empty PIN
    if (setLen == 0)
    {
        ddLcdClear();
        printf("PIN empty!");
        delay(FEEDBACK_DELAY_MS);
        return;
    }

    // Step 2: Ask user to confirm the PIN
    ddLcdClear();
    printf("Confirm PIN:");
    ddLcdSetCursor(0, 1);
    readPin(pinConfirm);

    // Step 3: Compare and show result
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

    // Wait before restarting the cycle
    delay(FEEDBACK_DELAY_MS);
}

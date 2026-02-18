// Lab 1.1 - LED control application via serial commands
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_1_1.h"
#include "dd_led/dd_led.h"
#include "srv_serial_stdio/srv_serial_stdio.h"

#define CMD_BUFFER_SIZE 64  // Maximum command length
#define APP_LED_PIN 12      // Pin connected to the LED
#define APP_LED_ID  0       // LED index in dd_led module

static char cmdBuffer[CMD_BUFFER_SIZE];  // Buffer for the incoming command
static int cmdIndex = 0;                 // Current position in the buffer

// Parse and execute the command stored in cmdBuffer
static void processCommand()
{
    // Null-terminate the command string
    cmdBuffer[cmdIndex] = '\0';

    // If empty command, just show prompt
    if (cmdIndex == 0)
    {
        printf("\n> ");
        return;
    }

    printf("\nReceived: %s\n", cmdBuffer);

    // Match command and execute corresponding LED action
    if (strcmp(cmdBuffer, "led on") == 0)
    {
        ddLedOn(APP_LED_ID);
        printf("OK: LED is ON\n");
    }
    else if (strcmp(cmdBuffer, "led off") == 0)
    {
        ddLedOff(APP_LED_ID);
        printf("OK: LED is OFF\n");
    }
    else if (strcmp(cmdBuffer, "led toggle") == 0)
    {
        ddLedToggle(APP_LED_ID);
        printf("OK: LED toggled -> %s\n", ddLedIsOn(APP_LED_ID) ? "ON" : "OFF");
    }
    else if (strcmp(cmdBuffer, "led status") == 0)
    {
        printf("OK: LED is %s\n", ddLedIsOn(APP_LED_ID) ? "ON" : "OFF");
    }
    else
    {
        printf("ERROR: Unknown command\n");
    }

    // Reset buffer and show prompt for next command
    cmdIndex = 0;
    printf("> ");
}

// Initialize serial communication and LED, print welcome message
void appLab11Setup()
{
    srvSerialSetup();
    ddLedSetup(APP_LED_ID, APP_LED_PIN);

    printf("Lab 1.1 - LED Control\n");
    printf("Type: led on | led off | led toggle | led status\n");
    printf("> ");
}

// Read one character at a time and build the command
void appLab11Loop()
{
    // Exit if no data available
    if (!Serial.available())
    {
        return;
    }

    int c = getchar();

    // Ignore carriage return
    if (c == '\r')
    {
        return;
    }
    // Newline means the command is complete
    else if (c == '\n')
    {
        processCommand();
    }
    // Handle backspace (ASCII 8 or 127)
    else if ((c == '\b' || c == 127) && cmdIndex > 0)
    {
        cmdIndex--;
        printf("\b \b");
    }
    // Append character to buffer and echo it back
    else if (cmdIndex < CMD_BUFFER_SIZE - 1)
    {
        cmdBuffer[cmdIndex++] = (char)c;
        putchar(c);
    }
}
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_1_1.h"
#include "dd_led/dd_led.h"
#include "srv_serial_stdio/srv_serial_stdio.h"

#define CMD_BUFFER_SIZE 64
#define APP_LED_PIN 12

static char cmdBuffer[CMD_BUFFER_SIZE];
static int cmdIndex = 0;

static void processCommand()
{
    cmdBuffer[cmdIndex] = '\0';

    if (cmdIndex == 0)
    {
        printf("\n> ");
        return;
    }

    printf("\nReceived: %s\n", cmdBuffer);

    if (strcmp(cmdBuffer, "led on") == 0)
    {
        ddLedOn();
        printf("OK: LED is ON\n");
    }
    else if (strcmp(cmdBuffer, "led off") == 0)
    {
        ddLedOff();
        printf("OK: LED is OFF\n");
    }
    else if (strcmp(cmdBuffer, "led toggle") == 0)
    {
        ddLedToggle();
        printf("OK: LED toggled -> %s\n", ddLedIsOn() ? "ON" : "OFF");
    }
    else if (strcmp(cmdBuffer, "led status") == 0)
    {
        printf("OK: LED is %s\n", ddLedIsOn() ? "ON" : "OFF");
    }
    else
    {
        printf("ERROR: Unknown command\n");
    }

    cmdIndex = 0;
    printf("> ");
}

void appLab11Setup()
{
    srvSerialSetup();
    ddLedSetup(APP_LED_PIN);

    printf("Lab 1.1 - LED Control\n");
    printf("Type: led on | led off | led toggle | led status\n");
    printf("> ");
}

void appLab11Loop()
{
    if (!Serial.available())
    {
        return;
    }

    int c = getchar();

    if (c == '\r')
    {
        return;
    }
    else if (c == '\n')
    {
        processCommand();
    }
    else if ((c == '\b' || c == 127) && cmdIndex > 0)
    {
        cmdIndex--;
        printf("\b \b");
    }
    else if (cmdIndex < CMD_BUFFER_SIZE - 1)
    {
        cmdBuffer[cmdIndex++] = (char)c;
        putchar(c);
    }
}
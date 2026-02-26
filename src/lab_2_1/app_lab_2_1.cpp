// Lab 2.1 - Button press duration monitor (non-preemptive bare-metal)
#include <Arduino.h>
#include <stdio.h>

#include "app_lab_2_1.h"
#include "dd_led/dd_led.h"
#include "dd_button/dd_button.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_os_sequential/srv_os_sequential.h"
#include "srv_heartbeat/srv_heartbeat.h"

#define PIN_BUTTON        2
#define PIN_LED_GREEN     3
#define PIN_LED_RED       4
#define PIN_LED_YELLOW    5
#define PIN_LED_HEARTBEAT 13

#define BTN_MAIN          0
#define LED_GREEN         0
#define LED_RED           1
#define LED_YELLOW        2
#define LED_HEARTBEAT     3

#define SHORT_PRESS_MAX_MS   500

// Task recurrences and offsets (in 1ms ticks)
#define REC_TASK1      10
#define REC_TASK2      50
#define REC_TASK3      10000
#define REC_HEARTBEAT  500

#define OFFS_TASK1     0
#define OFFS_TASK2     5
#define OFFS_TASK3     10
#define OFFS_HEARTBEAT 1

#define BLINK_SHORT_COUNT    5
#define BLINK_LONG_COUNT     10

// Shared state: Task 1 writes, Task 2 reads and clears
static volatile bool          newPressDetected   = false;
static volatile unsigned long lastPressDuration  = 0;
static volatile bool          lastPressWasShort  = false;

// Statistics: Task 2 updates, Task 3 reads and resets
static unsigned long totalPresses   = 0;
static unsigned long shortPresses   = 0;
static unsigned long longPresses    = 0;
static unsigned long totalDuration  = 0;

// Task 1 - Button detection and duration measurement

static bool          task1WasPressed = false;
static unsigned long task1PressStart = 0;

static void task1ButtonScan(void)
{
    ddButtonUpdate(BTN_MAIN);

    if (ddButtonPressed(BTN_MAIN))
    {
        task1WasPressed = true;
        task1PressStart = millis();
        ddLedOff(LED_GREEN);
        ddLedOff(LED_RED);
    }

    if (ddButtonReleased(BTN_MAIN) && task1WasPressed)
    {
        unsigned long duration = millis() - task1PressStart;
        task1WasPressed = false;

        lastPressDuration = duration;
        lastPressWasShort = (duration < SHORT_PRESS_MAX_MS);
        newPressDetected  = true;

        if (lastPressWasShort)
        {
            ddLedOn(LED_GREEN);
            ddLedOff(LED_RED);
        }
        else
        {
            ddLedOff(LED_GREEN);
            ddLedOn(LED_RED);
        }
    }
}

// Task 2 - Statistics counting + yellow LED blink state machine

static int  task2BlinkRemaining = 0;
static bool task2BlinkPhase     = false;

static void task2StatsAndBlink(void)
{
    if (newPressDetected)
    {
        newPressDetected = false;

        totalPresses++;
        totalDuration += lastPressDuration;

        if (lastPressWasShort)
        {
            shortPresses++;
            task2BlinkRemaining = BLINK_SHORT_COUNT * 2;
        }
        else
        {
            longPresses++;
            task2BlinkRemaining = BLINK_LONG_COUNT * 2;
        }

        task2BlinkPhase = false;
    }

    if (task2BlinkRemaining > 0)
    {
        task2BlinkPhase = !task2BlinkPhase;

        if (task2BlinkPhase)
        {
            ddLedOn(LED_YELLOW);
        }
        else
        {
            ddLedOff(LED_YELLOW);
        }

        task2BlinkRemaining--;
    }
    else
    {
        ddLedOff(LED_YELLOW);
    }
}

// Task 3 - Periodic reporting via STDIO

static void task3Report(void)
{
    if (totalPresses == 0)
    {
        printf("Report: no presses detected\n");
        return;
    }

    unsigned long avgDuration = totalDuration / totalPresses;

    printf("Periodic Report\n");
    printf("Total presses : %lu\n", totalPresses);
    printf("Short (<%dms)  : %lu\n", SHORT_PRESS_MAX_MS, shortPresses);
    printf("Long  (>=%dms) : %lu\n", SHORT_PRESS_MAX_MS, longPresses);
    printf("Avg duration   : %lu ms\n", avgDuration);

    totalPresses  = 0;
    shortPresses  = 0;
    longPresses   = 0;
    totalDuration = 0;
}

void appLab21Setup()
{
    srvSerialSetup();

    ddButtonSetup(BTN_MAIN, PIN_BUTTON);
    ddLedSetup(LED_GREEN,  PIN_LED_GREEN);
    ddLedSetup(LED_RED,    PIN_LED_RED);
    ddLedSetup(LED_YELLOW, PIN_LED_YELLOW);

    srvHeartbeatSetup(LED_HEARTBEAT, PIN_LED_HEARTBEAT);

    task1WasPressed     = false;
    task1PressStart     = 0;
    task2BlinkRemaining = 0;
    task2BlinkPhase     = false;

    srvOsSetup();
    srvOsRegisterTask(task1ButtonScan,   REC_TASK1,     OFFS_TASK1);
    srvOsRegisterTask(task2StatsAndBlink, REC_TASK2,     OFFS_TASK2);
    srvOsRegisterTask(task3Report,        REC_TASK3,     OFFS_TASK3);
    srvOsRegisterTask(srvHeartbeatTask,   REC_HEARTBEAT, OFFS_HEARTBEAT);

    printf("Lab 2.1 - Button Duration Monitor\n");
    printf("Press the button. Reports every 10 s.\n");
}

void appLab21Loop()
{
    srvOsSchedulerLoop();
}

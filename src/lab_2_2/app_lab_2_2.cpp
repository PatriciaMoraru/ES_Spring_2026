// Lab 2.2 - Button press duration monitor (preemptive FreeRTOS)
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>

#include "app_lab_2_2.h"
#include "dd_led/dd_led.h"
#include "dd_button/dd_button.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"

// --- Pin assignments ---
// Pins 2,3,5 are Timer 3 OC pins on ATmega2560 (used by FreeRTOS tick).
#define PIN_BUTTON 6
#define PIN_LED_GREEN 7
#define PIN_LED_RED 8
#define PIN_LED_YELLOW 9
#define PIN_LED_HEARTBEAT 13

// --- Logical IDs for device drivers ---
#define BTN_MAIN 0
#define LED_GREEN 0
#define LED_RED 1
#define LED_YELLOW 2
#define LED_HEARTBEAT 3

// Short/long press threshold
#define SHORT_PRESS_MAX_MS 500

// Guarantees at least 1 tick even at low tick rates (e.g. 62 Hz)
#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)

// --- Task scheduling configuration (ms) ---
#define PERIOD_TASK1 10    // Button scan
#define PERIOD_TASK2 50    // Stats + blink
#define PERIOD_TASK3 10000 // STDIO report
#define PERIOD_HEARTBEAT 500

#define OFFSET_TASK1 0
#define OFFSET_TASK2 5
#define OFFSET_TASK3 10
#define OFFSET_HEARTBEAT 1

// Yellow LED blink counts per press type
#define BLINK_SHORT_COUNT 5
#define BLINK_LONG_COUNT 10

// Rapid-press streak detection
#define STREAK_WINDOW_TICKS MS_TO_TICKS(1000)
#define STREAK_THRESHOLD 3
#define CELEBRATION_FLASHES 4

// --- Synchronization primitives ---
static SemaphoreHandle_t xPressSem = NULL;    // Binary sem: Task1 -> Task2 event
static SemaphoreHandle_t xPressMutex = NULL;  // Protects press event data
static SemaphoreHandle_t xStatsMutex = NULL;  // Protects statistics counters
static SemaphoreHandle_t xStreakMutex = NULL; // Protects streak data

// Press event data (Task1 writes, Task2 reads)
static unsigned long sharedPressDuration = 0;
static bool sharedPressWasShort = false;

// Statistics (Task2 updates, Task3 reads and resets)
static unsigned long totalPresses = 0;
static unsigned long shortPresses = 0;
static unsigned long longPresses = 0;
static unsigned long totalDuration = 0;

// Streak tracking (Task2 updates, Task3 reads bestStreak)
static TickType_t streakCount = 0;
static TickType_t lastPressTick = 0;
static TickType_t bestStreak = 0;

// Task 1 - Button detection and duration measurement (10ms, priority 3)
// Polls the debounced button driver, measures press duration on release,
// writes result to shared variables, and signals Task 2 via binary semaphore.
static void task1ButtonScan(void *pvParameters)
{
    (void)pvParameters;

    if (OFFSET_TASK1 > 0)
    {
        vTaskDelay(MS_TO_TICKS(OFFSET_TASK1));
    }

    TickType_t xLastWake = xTaskGetTickCount();

    bool wasHeld = false;
    TickType_t pressStartTick = 0;

    for (;;)
    {
        ddButtonUpdate(BTN_MAIN);
        bool held = ddButtonIsPressed(BTN_MAIN);

        // Press edge — record start tick, clear LEDs
        if (held && !wasHeld)
        {
            pressStartTick = xTaskGetTickCount();
            ddLedOff(LED_GREEN);
            ddLedOff(LED_RED);
        }

        // Release edge — compute duration, signal Task 2
        if (!held && wasHeld)
        {
            unsigned long durationMs =
                (xTaskGetTickCount() - pressStartTick) * portTICK_PERIOD_MS;

            bool isShort = (durationMs < SHORT_PRESS_MAX_MS);

            // Write press data under mutex, then signal Task 2
            xSemaphoreTake(xPressMutex, portMAX_DELAY);
            sharedPressDuration = durationMs;
            sharedPressWasShort = isShort;
            xSemaphoreGive(xPressMutex);

            // Sendins a signal to notify Task 2 an event is ready
            xSemaphoreGive(xPressSem);

            // Visual feedback: green = short, red = long
            if (isShort)
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

        wasHeld = held;

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_TASK1));
    }
}

// Task 2 - Statistics counting + yellow LED blink (50ms, priority 2)
// Waits for press events (non-blocking semaphore take), updates counters,
// tracks rapid-press streaks, and drives the yellow LED blink animation.
static void task2StatsAndBlink(void *pvParameters)
{
    (void)pvParameters;

    if (OFFSET_TASK2 > 0)
    {
        vTaskDelay(MS_TO_TICKS(OFFSET_TASK2));
    }

    TickType_t xLastWake = xTaskGetTickCount();

    int blinkRemaining = 0; // blink half-cycles left (on+off = 2)
    bool blinkPhase = false;
    bool celebrating = false; // all-LED celebration mode

    for (;;)
    {
        // --- Process press event (if any) ---
        if (xSemaphoreTake(xPressSem, 0) == pdTRUE)
        {
            // Read press data under mutex
            xSemaphoreTake(xPressMutex, portMAX_DELAY);
            unsigned long duration = sharedPressDuration;
            bool isShort = sharedPressWasShort;
            xSemaphoreGive(xPressMutex);

            // Update statistics under mutex
            xSemaphoreTake(xStatsMutex, portMAX_DELAY);
            totalPresses++;
            totalDuration += duration;

            if (isShort)
                shortPresses++;
            else
                longPresses++;
            xSemaphoreGive(xStatsMutex);

            // Streak: count rapid presses within the time window
            TickType_t now = xTaskGetTickCount();

            xSemaphoreTake(xStreakMutex, portMAX_DELAY);
            if (lastPressTick != 0 && (now - lastPressTick) < STREAK_WINDOW_TICKS)
            {
                streakCount++;
            }
            else
            {
                streakCount = 1;
            }
            lastPressTick = now;

            if (streakCount > bestStreak)
            {
                bestStreak = streakCount;
            }

            // Trigger celebration once per streak crossing
            bool streakHit = (streakCount >= STREAK_THRESHOLD && !celebrating);
            xSemaphoreGive(xStreakMutex);

            if (streakHit)
            {
                celebrating = true;
            }

            // Set blink count: celebration overrides normal blink
            if (celebrating)
            {
                blinkRemaining = CELEBRATION_FLASHES * 2;
            }
            else if (isShort)
            {
                blinkRemaining = BLINK_SHORT_COUNT * 2;
            }
            else
            {
                blinkRemaining = BLINK_LONG_COUNT * 2;
            }

            blinkPhase = false;
        }

        // --- Blink animation ---
        if (blinkRemaining > 0)
        {
            blinkPhase = !blinkPhase;

            if (celebrating)
            {
                // All 3 LEDs flash together
                if (blinkPhase)
                {
                    ddLedOn(LED_GREEN);
                    ddLedOn(LED_RED);
                    ddLedOn(LED_YELLOW);
                }
                else
                {
                    ddLedOff(LED_GREEN);
                    ddLedOff(LED_RED);
                    ddLedOff(LED_YELLOW);
                }
            }
            else
            {
                // Normal: only yellow blinks
                if (blinkPhase)
                {
                    ddLedOn(LED_YELLOW);
                }
                else
                {
                    ddLedOff(LED_YELLOW);
                }
            }

            blinkRemaining--;

            if (blinkRemaining == 0)
            {
                if (celebrating)
                {
                    ddLedOff(LED_GREEN);
                    ddLedOff(LED_RED);
                }
                celebrating = false;
            }
        }
        else
        {
            ddLedOff(LED_YELLOW);
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_TASK2));
    }
}

// Task 3 - Periodic reporting via STDIO (10s, priority 1)
// Snapshots and resets all statistics under mutex, then prints the report.
static void task3Report(void *pvParameters)
{
    (void)pvParameters;

    if (OFFSET_TASK3 > 0)
    {
        vTaskDelay(MS_TO_TICKS(OFFSET_TASK3));
    }

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        unsigned long tp, sp, lp, td;
        TickType_t bs;

        // Snapshot and reset stats atomically
        xSemaphoreTake(xStatsMutex, portMAX_DELAY);
        tp = totalPresses;
        sp = shortPresses;
        lp = longPresses;
        td = totalDuration;
        totalPresses = 0;
        shortPresses = 0;
        longPresses = 0;
        totalDuration = 0;
        xSemaphoreGive(xStatsMutex);

        xSemaphoreTake(xStreakMutex, portMAX_DELAY);
        bs = bestStreak;
        bestStreak = 0;
        xSemaphoreGive(xStreakMutex);

        if (tp == 0)
        {
            printf("Report: no presses detected\n");
        }
        else
        {
            unsigned long avg = td / tp;

            printf("Periodic Report\n");
            printf("Total presses : %lu\n", tp);
            printf("Short (<%dms)  : %lu\n", SHORT_PRESS_MAX_MS, sp);
            printf("Long  (>=%dms) : %lu\n", SHORT_PRESS_MAX_MS, lp);
            printf("Avg duration   : %lu ms\n", avg);
            printf("Best streak    : %u rapid presses\n", (unsigned)bs);
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_TASK3));
    }
}

// Heartbeat task (500ms, priority 1) — toggles LED 13 to show scheduler is alive
static void taskHeartbeat(void *pvParameters)
{
    (void)pvParameters;

    if (OFFSET_HEARTBEAT > 0)
    {
        vTaskDelay(MS_TO_TICKS(OFFSET_HEARTBEAT));
    }

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        srvHeartbeatTask();
        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_HEARTBEAT));
    }
}

// Setup — initializes HW, creates RTOS objects, starts scheduler
void appLab22Setup()
{
    srvSerialSetup();

    // Hardware init: button (INPUT_PULLUP) and LEDs (OUTPUT)
    ddButtonSetup(BTN_MAIN, PIN_BUTTON);
    ddLedSetup(LED_GREEN, PIN_LED_GREEN);
    ddLedSetup(LED_RED, PIN_LED_RED);
    ddLedSetup(LED_YELLOW, PIN_LED_YELLOW);

    srvHeartbeatSetup(LED_HEARTBEAT, PIN_LED_HEARTBEAT);

    // Create synchronization primitives
    xPressSem = xSemaphoreCreateBinary();
    xPressMutex = xSemaphoreCreateMutex();
    xStatsMutex = xSemaphoreCreateMutex();
    xStreakMutex = xSemaphoreCreateMutex();

    if (!xPressSem || !xPressMutex || !xStatsMutex || !xStreakMutex)
    {
        printf("FATAL: semaphore creation failed (heap too small?)\n");
        for (;;)
        {
        }
    }

    // Create tasks: higher number = higher priority
    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(task1ButtonScan, "BtnScan", 256, NULL, 3, NULL);
    ok &= xTaskCreate(task2StatsAndBlink, "Stats", 256, NULL, 2, NULL);
    ok &= xTaskCreate(task3Report, "Report", 512, NULL, 1, NULL);
    ok &= xTaskCreate(taskHeartbeat, "HB", 192, NULL, 1, NULL);

    if (ok != pdPASS)
    {
        printf("FATAL: task creation failed (heap too small?)\n");
        for (;;)
        {
        }
    }

    printf("Lab 2.2 - Button Duration Monitor (FreeRTOS)\n");
    printf("Press the button. Reports every 10 s.\n");

    // Scheduler never returns on success
    vTaskStartScheduler();
}

void appLab22Loop()
{
    // FreeRTOS scheduler is running; this is never reached.
}

// Lab 4.1 - Binary Actuator Control (FreeRTOS, Variant B)
//
// Actuator : relay on PIN_ACTUATOR (active-LOW) — switches a red LED on the breadboard
// Input    : Serial STDIO via scanf() — on / off / toggle / timer N / status / report / help
// Output   : I2C LCD 1602 + Serial monitor
//
// FreeRTOS tasks:
//   1. taskCmdRead            - scanf() token parsing, immediate help/status reply    (event-driven, prio 1)
//   2. taskActuatorControl    - consumes pending command, auto-off timer, drives relay (75 ms,  prio 3)
//   3. taskSignalConditioning - saturation, SW-debounce, false-switch detection        (100 ms, prio 2)
//   4. taskDisplay            - LCD pages + on-demand serial report + Teleplot         (500 ms, prio 1)
//   5. taskHeartbeat          - heartbeat blink                                        (500 ms, prio 1)

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_4_1.h"
#include "dd_actuator/dd_actuator.h"
#include "dd_led/dd_led.h"
#include "dd_lcd/dd_lcd.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"


// Pin assignments  (pins 2, 3, 5 reserved by FreeRTOS Timer 3)


#define PIN_ACTUATOR      6
#define PIN_LED_HEARTBEAT 13

#define LCD_I2C_ADDR  0x27
#define LCD_COLS      16
#define LCD_ROWS      2

#define ACT_ID        0
#define LED_HEARTBEAT 0

#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)


// Task periods (ms)


#define PERIOD_ACT_CTRL  75
#define PERIOD_CONDITION 100
#define PERIOD_DISPLAY   500
#define PERIOD_HEARTBEAT 500


// Signal conditioning parameters


// Minimum ms between two accepted state changes (command rate-limiting)
#define CMD_DEBOUNCE_MS  150

// Consecutive conditioning cycles required before committing a new validated state
#define DEBOUNCE_COUNT   3

// False-switch detection: > N transitions within W ms triggers the flag
#define FALSE_SWITCH_THRESHOLD   5
#define FALSE_SWITCH_WINDOW_MS   5000

// Maximum seconds allowed for the timer command
#define TIMER_MAX_SEC  3600

// LCD page rotation: switch every N display cycles
#define LCD_PAGE_CYCLES  3


// Shared data types


// Command types routed from taskCmdRead to taskActuatorControl
typedef enum
{
    CMD_NONE,
    CMD_ON,
    CMD_OFF,
    CMD_TOGGLE,
    CMD_TIMER,    // turn ON for timerSec seconds then auto-off
    CMD_REPORT
} CmdType_t;

// Written by taskCmdRead; consumed by taskActuatorControl
typedef struct
{
    CmdType_t type;
    uint16_t  timerSec;   // only meaningful for CMD_TIMER
} PendingCmd_t;  // protected by xCmdMutex

// Written by taskActuatorControl; read by taskSignalConditioning + taskDisplay
typedef struct
{
    bool     rawState;
    uint8_t  cmdCount;
    uint32_t lastChangedMs;
    char     lastCmd[8];
    bool     timerActive;      // auto-off timer is running
    uint16_t timerRemainSec;   // approx. seconds until auto-off
} ActuatorData_t;

// Written by taskSignalConditioning; read by taskDisplay
typedef struct
{
    bool    validatedState;
    bool    falseSwitchFlag;
    uint8_t falseSwitchCount;
    uint8_t debounceCounter;
} ConditionedData_t;


// Shared instances + mutexes


static PendingCmd_t      pendingCmd      = {CMD_NONE, 0};
static ActuatorData_t    actuatorData    = {false, 0, 0, "none", false, 0};
static ConditionedData_t conditionedData = {false, false, 0, 0};

static SemaphoreHandle_t xCmdMutex  = NULL;   // protects pendingCmd
static SemaphoreHandle_t xActMutex  = NULL;   // protects actuatorData
static SemaphoreHandle_t xCondMutex = NULL;   // protects conditionedData

// Set by taskActuatorControl on CMD_REPORT; cleared by taskDisplay.
// Single-byte write is atomic on AVR — no mutex needed.
static volatile bool reportRequested = false;


// Task 1 -- Command Reader  (event-driven, priority 1)
//
// Blocks on scanf() waiting for serial tokens. Properly yields via the
// vTaskDelay(1) inside srvSerialGetChar (SERIAL_FREERTOS_YIELD build flag),
// so all higher-priority tasks run freely while no input is pending.
//
// "help" and "status" are answered immediately here — they don't need to
// go through the actuator control pipeline.
// All other commands are written to PendingCmd_t under xCmdMutex for
// taskActuatorControl to consume at its next 75 ms cycle.


static void taskCmdRead(void *pvParameters)
{
    (void)pvParameters;

    char cmd[8];

    for (;;)
    {
        if (scanf("%7s", cmd) != 1)
            continue;

        // Normalise to lower-case in-place
        for (uint8_t i = 0; cmd[i]; i++)
            if (cmd[i] >= 'A' && cmd[i] <= 'Z')
                cmd[i] += 32;

        // --- commands answered directly (no pipeline) ---

        if (strcmp(cmd, "help") == 0)
        {
            printf("Commands:\n");
            printf("  on       - turn actuator ON\n");
            printf("  off      - turn actuator OFF\n");
            printf("  toggle   - flip current state\n");
            printf("  timer N  - turn ON for N seconds then auto-off (max %d)\n", TIMER_MAX_SEC);
            printf("  status   - print current raw state\n");
            printf("  report   - print full conditioning report\n");
            printf("  help     - show this list\n");
            continue;
        }

        if (strcmp(cmd, "status") == 0)
        {
            xSemaphoreTake(xActMutex, portMAX_DELAY);
            bool st      = actuatorData.rawState;
            bool tmr     = actuatorData.timerActive;
            uint16_t rem = actuatorData.timerRemainSec;
            xSemaphoreGive(xActMutex);
            printf("[STATUS] Actuator is %s", st ? "ON" : "OFF");
            if (tmr)
                printf("  (auto-off in %us)", rem);
            printf("\n");
            continue;
        }

        // --- commands routed through the pipeline ---

        CmdType_t type     = CMD_NONE;
        uint16_t  timerSec = 0;

        if (strcmp(cmd, "on")     == 0) type = CMD_ON;
        else if (strcmp(cmd, "off")    == 0) type = CMD_OFF;
        else if (strcmp(cmd, "toggle") == 0) type = CMD_TOGGLE;
        else if (strcmp(cmd, "report") == 0) type = CMD_REPORT;
        else if (strcmp(cmd, "timer")  == 0)
        {
            unsigned int sec = 0;
            if (scanf("%u", &sec) == 1 && sec > 0 && sec <= TIMER_MAX_SEC)
            {
                timerSec = (uint16_t)sec;
                type     = CMD_TIMER;
            }
            else
            {
                printf("[ERR] Usage: timer <seconds>  (1 – %d)\n", TIMER_MAX_SEC);
                continue;
            }
        }
        else
        {
            printf("[ERR] Unknown command '%s'. Type 'help' for the list.\n", cmd);
            continue;
        }

        xSemaphoreTake(xCmdMutex, portMAX_DELAY);
        pendingCmd.type     = type;
        pendingCmd.timerSec = timerSec;
        xSemaphoreGive(xCmdMutex);
    }
}


// Task 2 -- Actuator Control  (periodic 75 ms, priority 3)
//
// At each cycle:
//   1. Consumes any pending command from PendingCmd_t (with rate-limit debounce).
//   2. Checks whether the auto-off timer has expired and turns the actuator off.
//   3. Drives dd_actuator and writes the full ActuatorData_t snapshot.


static void taskActuatorControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint32_t lastCmdMs  = 0;
    uint32_t autoOffMs  = 0;
    bool     timerActive = false;

    for (;;)
    {
        // --- consume pending command ---
        xSemaphoreTake(xCmdMutex, portMAX_DELAY);
        PendingCmd_t cmd = pendingCmd;
        pendingCmd.type  = CMD_NONE;
        xSemaphoreGive(xCmdMutex);

        if (cmd.type == CMD_REPORT)
        {
            reportRequested = true;
        }
        else if (cmd.type != CMD_NONE)
        {
            uint32_t now = (uint32_t)millis();
            if ((now - lastCmdMs) < CMD_DEBOUNCE_MS)
            {
                printf("[ACT] command ignored (debounce)\n");
            }
            else
            {
                lastCmdMs = now;

                bool cur      = ddActuatorGetState(ACT_ID);
                bool newState = cur;
                const char *label = "?";

                if (cmd.type == CMD_ON)
                {
                    newState = true;
                    label    = "on";
                    timerActive = false;
                }
                else if (cmd.type == CMD_OFF)
                {
                    newState    = false;
                    label       = "off";
                    timerActive = false;
                }
                else if (cmd.type == CMD_TOGGLE)
                {
                    newState    = !cur;
                    label       = "toggle";
                    timerActive = false;
                }
                else if (cmd.type == CMD_TIMER)
                {
                    newState    = true;
                    label       = "timer";
                    autoOffMs   = now + (uint32_t)cmd.timerSec * 1000UL;
                    timerActive = true;
                    printf("[ACT] timer started — auto-off in %us\n", cmd.timerSec);
                }

                if (newState) ddActuatorOn(ACT_ID);
                else          ddActuatorOff(ACT_ID);

                if (cmd.type != CMD_TIMER)
                    printf("[ACT] %s -> %s\n", label, newState ? "ON" : "OFF");

                uint32_t changeMs = now;

                uint16_t remainSec = 0;
                if (timerActive)
                {
                    uint32_t remaining = autoOffMs - now;
                    remainSec = (uint16_t)(remaining / 1000UL);
                }

                xSemaphoreTake(xActMutex, portMAX_DELAY);
                actuatorData.rawState       = newState;
                actuatorData.cmdCount++;
                actuatorData.lastChangedMs  = changeMs;
                actuatorData.timerActive    = timerActive;
                actuatorData.timerRemainSec = remainSec;
                strncpy(actuatorData.lastCmd, label, sizeof(actuatorData.lastCmd) - 1);
                actuatorData.lastCmd[sizeof(actuatorData.lastCmd) - 1] = '\0';
                xSemaphoreGive(xActMutex);
            }
        }

        // --- check auto-off timer ---
        if (timerActive)
        {
            uint32_t now = (uint32_t)millis();
            if (now >= autoOffMs)
            {
                timerActive = false;
                ddActuatorOff(ACT_ID);

                xSemaphoreTake(xActMutex, portMAX_DELAY);
                actuatorData.rawState       = false;
                actuatorData.cmdCount++;
                actuatorData.lastChangedMs  = now;
                actuatorData.timerActive    = false;
                actuatorData.timerRemainSec = 0;
                strncpy(actuatorData.lastCmd, "t-exp", sizeof(actuatorData.lastCmd) - 1);
                xSemaphoreGive(xActMutex);

                printf("[ACT] timer expired -> OFF\n");
            }
            else
            {
                // Update remaining seconds each cycle for display
                uint16_t rem = (uint16_t)((autoOffMs - now) / 1000UL);
                xSemaphoreTake(xActMutex, portMAX_DELAY);
                actuatorData.timerRemainSec = rem;
                xSemaphoreGive(xActMutex);
            }
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_ACT_CTRL));
    }
}


// Task 3 -- Signal Conditioning  (priority 2, 100 ms)
//
// Reads ActuatorData_t snapshot and runs three conditioning stages:
//   1. Saturation  : clamp rawState to {0, 1} — enforces valid boolean domain
//   2. SW-debounce : only commit a new validatedState after DEBOUNCE_COUNT
//                    consecutive cycles all agree on the same value
//   3. False-switch: count distinct state transitions within FALSE_SWITCH_WINDOW_MS;
//                    set falseSwitchFlag if the rate exceeds the threshold
// Results are written to ConditionedData_t and shown on LCD / serial report.


static void taskSignalConditioning(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    // Debounce state
    bool    pendingState    = false;
    uint8_t debounceCounter = 0;
    bool    lastValidated   = false;

    // False-switch sliding window (ring buffer)
    static uint32_t transitionTs[FALSE_SWITCH_THRESHOLD + 2];
    uint8_t tsHead  = 0;
    uint8_t tsCount = 0;

    bool    falseSwitchFlag  = false;
    uint8_t falseSwitchCount = 0;

    bool lastRaw = false;

    for (;;)
    {
        // --- snapshot ---
        ActuatorData_t snap;
        xSemaphoreTake(xActMutex, portMAX_DELAY);
        snap = actuatorData;
        xSemaphoreGive(xActMutex);

        // --- Stage 1: saturation (clamp to valid boolean) ---
        bool saturated = snap.rawState ? true : false;

        // --- Stage 2: software debounce ---
        if (saturated == pendingState)
        {
            if (debounceCounter < DEBOUNCE_COUNT)
                debounceCounter++;
        }
        else
        {
            pendingState    = saturated;
            debounceCounter = 1;
        }

        bool newValidated = lastValidated;
        if (debounceCounter >= DEBOUNCE_COUNT)
            newValidated = pendingState;

        // --- Stage 3: false-switch detection ---
        if (saturated != lastRaw)
        {
            uint32_t now = (uint32_t)millis();
            transitionTs[tsHead] = now;
            tsHead  = (tsHead + 1) % (uint8_t)(FALSE_SWITCH_THRESHOLD + 2);
            if (tsCount < (uint8_t)(FALSE_SWITCH_THRESHOLD + 2))
                tsCount++;

            uint8_t recent = 0;
            for (uint8_t i = 0; i < tsCount; i++)
            {
                uint8_t idx = (tsHead + (FALSE_SWITCH_THRESHOLD + 2) - tsCount + i)
                              % (uint8_t)(FALSE_SWITCH_THRESHOLD + 2);
                if ((now - transitionTs[idx]) <= FALSE_SWITCH_WINDOW_MS)
                    recent++;
            }

            if (recent > FALSE_SWITCH_THRESHOLD)
            {
                if (!falseSwitchFlag)
                    falseSwitchCount++;
                falseSwitchFlag = true;
                printf("[COND] FALSE-SWITCH ALERT! %d transitions in %d ms\n",
                       recent, FALSE_SWITCH_WINDOW_MS);
            }
            else
            {
                falseSwitchFlag = false;
            }

            lastRaw = saturated;
        }

        lastValidated = newValidated;

        // --- write conditioned data ---
        xSemaphoreTake(xCondMutex, portMAX_DELAY);
        conditionedData.validatedState   = newValidated;
        conditionedData.falseSwitchFlag  = falseSwitchFlag;
        conditionedData.falseSwitchCount = falseSwitchCount;
        conditionedData.debounceCounter  = debounceCounter;
        xSemaphoreGive(xCondMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONDITION));
    }
}


// Task 4 -- Display & Reporting  (priority 1, 500 ms)
//
// LCD alternates between two pages every LCD_PAGE_CYCLES cycles:
//   Page 0: raw actuator state + validated state  /  command count + false-switch count
//   Page 1: time in current state (or countdown if timer active)  /  alert count
// Serial: full structured report on demand (triggered by "report" command).


static void taskDisplay(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t cycleCount  = 0;
    uint8_t currentPage = 0;

    for (;;)
    {
        // --- snapshots ---
        ActuatorData_t    snapA;
        ConditionedData_t snapC;

        xSemaphoreTake(xActMutex, portMAX_DELAY);
        snapA = actuatorData;
        xSemaphoreGive(xActMutex);

        xSemaphoreTake(xCondMutex, portMAX_DELAY);
        snapC = conditionedData;
        xSemaphoreGive(xCondMutex);

        uint32_t stateAgeMs  = (uint32_t)millis() - snapA.lastChangedMs;
        uint16_t stateAgeSec = (uint16_t)(stateAgeMs / 1000);

        // --- LCD page rotation ---
        cycleCount++;
        if (cycleCount >= LCD_PAGE_CYCLES)
        {
            cycleCount  = 0;
            currentPage = (currentPage + 1) % 2;
        }

        char row0[17];
        char row1[17];

        if (currentPage == 0)
        {
            // Page 0: ACT:ON/OFF  V:ON/OFF  /  Cmds:N  SW:M
            snprintf(row0, sizeof(row0), "ACT:%-3s V:%-3s",
                     snapA.rawState       ? "ON" : "OFF",
                     snapC.validatedState ? "ON" : "OFF");
            snprintf(row1, sizeof(row1), "Cmds:%-3u SW:%-3u",
                     snapA.cmdCount, snapC.falseSwitchCount);
        }
        else
        {
            // Page 1: state age or timer countdown  /  alert count
            if (snapA.timerActive)
                snprintf(row0, sizeof(row0), "OFF in: %us", snapA.timerRemainSec);
            else
                snprintf(row0, sizeof(row0), "State: %us", stateAgeSec);

            snprintf(row1, sizeof(row1), "Alerts: %-3u", snapC.falseSwitchCount);
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(row0);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(row1);

        // --- on-demand serial report (triggered by "report" command) ---
        if (reportRequested)
        {
            reportRequested = false;

            printf("--- Actuator Report ---\n");
            printf("Raw state   : %s\n", snapA.rawState       ? "ON" : "OFF");
            printf("Validated   : %s\n", snapC.validatedState  ? "ON" : "OFF");
            printf("Last cmd    : %s\n", snapA.lastCmd);
            printf("Cmd count   : %u\n", snapA.cmdCount);
            printf("State age   : %us\n", stateAgeSec);
            if (snapA.timerActive)
                printf("Timer       : auto-off in %us\n", snapA.timerRemainSec);
            else
                printf("Timer       : inactive\n");
            printf("Debounce cnt: %u / %d\n", snapC.debounceCounter, DEBOUNCE_COUNT);
            if (snapC.falseSwitchFlag)
                printf("ALERT       : false-switch detected! (total=%u)\n",
                       snapC.falseSwitchCount);
            else
                printf("Alert       : none\n");

            printf(">actState:%d\n",    (int)snapA.rawState);
            printf(">validState:%d\n",  (int)snapC.validatedState);
            printf(">cmdCount:%u\n",    snapA.cmdCount);
            printf(">falseSwitch:%d\n", (int)snapC.falseSwitchFlag);
            printf(">stateAge:%u\n",    stateAgeSec);
            printf(">timerRemain:%u\n", snapA.timerRemainSec);
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_DISPLAY));
    }
}


// Task 5 -- Heartbeat  (priority 1, 500 ms)


static void taskHeartbeat(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        srvHeartbeatTask();
        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_HEARTBEAT));
    }
}


// Setup


void appLab41Setup()
{
    srvSerialSetup();

    ddActuatorSetup(ACT_ID, PIN_ACTUATOR, true);  // true = active-LOW relay module

    srvHeartbeatSetup(LED_HEARTBEAT, PIN_LED_HEARTBEAT);

    ddLcdSetup(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

    xCmdMutex  = xSemaphoreCreateMutex();
    xActMutex  = xSemaphoreCreateMutex();
    xCondMutex = xSemaphoreCreateMutex();

    if (!xCmdMutex || !xActMutex || !xCondMutex)
    {
        printf("FATAL: mutex creation failed\n");
        for (;;) { }
    }

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(taskCmdRead,            "CmdRead",   256, NULL, 1, NULL);
    ok &= xTaskCreate(taskActuatorControl,    "ActCtrl",   320, NULL, 3, NULL);
    ok &= xTaskCreate(taskSignalConditioning, "Condition", 320, NULL, 2, NULL);
    ok &= xTaskCreate(taskDisplay,            "Display",   384, NULL, 1, NULL);
    ok &= xTaskCreate(taskHeartbeat,          "HB",        192, NULL, 1, NULL);

    if (ok != pdPASS)
    {
        printf("FATAL: task creation failed\n");
        for (;;) { }
    }

    printf("Lab 4.1 - Binary Actuator Control (Variant B)\n");
    printf("Actuator : PIN %d (active-LOW relay/LED)\n", PIN_ACTUATOR);
    printf("Type 'help' for available commands.\n");
    printf("Periods  : ActCtrl=%dms  Cond=%dms  Display=%dms\n",
           PERIOD_ACT_CTRL, PERIOD_CONDITION, PERIOD_DISPLAY);
    printf("Debounce : %dms cmd rate-limit | %d cycles SW-debounce\n",
           CMD_DEBOUNCE_MS, DEBOUNCE_COUNT);

    vTaskStartScheduler();
}

void appLab41Loop()
{
    // FreeRTOS scheduler is running; never reached.
}

// Lab 5.1 — ON-OFF Position Control with Hysteresis (FreeRTOS, Variant B)
//
// Sensor   : potentiometer on PIN_POT (analog) — rotor position feedback
// Actuator : DC motor via L298N (ENA=PWM, IN1, IN2) — fixed 50% power
// Input    : 2 buttons (UP/DOWN) for setpoint + Serial STDIO commands
// Output   : I2C LCD 1602 + Serial monitor
//
// Control  : ON-OFF with hysteresis (dead-band)
//   position < setPoint - hyst  →  motor FWD  (50%)
//   position > setPoint + hyst  →  motor REV  (50%)
//   within dead-band            →  motor STOP
//
// FreeRTOS tasks:
//   1. taskSerialInput  - scanf() command parsing: sp, hyst, status, report  (event,  prio 1)
//   2. taskButtonInput  - UP/DOWN buttons adjust setpoint with debounce      (50 ms,  prio 2)
//   3. taskAcquisition  - read potentiometer, median filter                  (50 ms,  prio 2)
//   4. taskControl      - ON-OFF hysteresis logic, drive motor               (100 ms, prio 3)
//   5. taskDisplay      - LCD pages + serial report on demand                (500 ms, prio 1)
//   6. taskHeartbeat    - heartbeat blink                                    (500 ms, prio 1)

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_5_1.h"
#include "dd_motor/dd_motor.h"
#include "dd_button/dd_button.h"
#include "dd_lcd/dd_lcd.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"


// Pin assignments  (pins 2, 3, 5 reserved by FreeRTOS Timer 3)


#define PIN_POT           A0     // potentiometer wiper — position feedback
#define PIN_BTN_UP        10     // setpoint increment button (INPUT_PULLUP)
#define PIN_BTN_DOWN      11     // setpoint decrement button (INPUT_PULLUP)
#define PIN_MOTOR_EN      9      // L298N ENA — PWM
#define PIN_MOTOR_IN1     7      // L298N IN1
#define PIN_MOTOR_IN2     8      // L298N IN2
#define PIN_LED_HEARTBEAT 13

#define LCD_I2C_ADDR  0x27
#define LCD_COLS      16
#define LCD_ROWS      2

#define MOTOR_ID      0
#define BTN_UP_ID     0
#define BTN_DOWN_ID   1
#define LED_HB_ID     0

#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)


// Task periods (ms)


#define PERIOD_BUTTON     50
#define PERIOD_ACQ        50
#define PERIOD_CONTROL    100
#define PERIOD_DISPLAY    500
#define PERIOD_HEARTBEAT  500


// Control parameters


#define MOTOR_FIXED_PCT   50       // fixed saturation speed (%)
#define SP_DEFAULT        512      // default setpoint (0-1023)
#define HYST_DEFAULT      30       // default hysteresis band (ADC units)
#define SP_MIN            0
#define SP_MAX            1023
#define HYST_MIN          5
#define HYST_MAX          200
#define SP_BTN_STEP       10       // setpoint change per button press

#define MEDIAN_BUF_SIZE   5        // samples for median filter
#define LCD_PAGE_CYCLES   3        // display cycles per page


// Control output states


typedef enum
{
    CTRL_STOP,
    CTRL_FWD,
    CTRL_REV
} CtrlAction_t;


// Shared data types


typedef struct
{
    int16_t setPoint;
    int16_t hysteresis;
} SetPointData_t;

typedef struct
{
    int16_t rawPosition;       // latest analogRead (0-1023)
    int16_t filteredPosition;  // after median filter
} AcqData_t;

typedef struct
{
    CtrlAction_t action;       // STOP / FWD / REV
    int16_t      error;        // setPoint - filteredPosition
    uint32_t     lastChangeMs; // timestamp of last action change
    uint16_t     switchCount;  // total direction changes
} ControlData_t;


// Shared instances + mutexes


static SetPointData_t spData   = {SP_DEFAULT, HYST_DEFAULT};
static AcqData_t      acqData  = {0, 0};
static ControlData_t  ctrlData = {CTRL_STOP, 0, 0, 0};

static SemaphoreHandle_t xSpMutex   = NULL;
static SemaphoreHandle_t xAcqMutex  = NULL;
static SemaphoreHandle_t xCtrlMutex = NULL;

static volatile bool reportRequested = false;


// Task 1 -- Serial command interpreter (event-driven, priority 1)
//
// Blocks on scanf() waiting for tokens. Supported commands:
//   sp N    — set setpoint (0-1023)
//   hyst N  — set hysteresis band (5-200)
//   status  — print current control state
//   report  — request detailed report from display task
//   help    — show command list


static void toLower(char *s)
{
    while (*s) { if (*s >= 'A' && *s <= 'Z') *s |= 0x20; s++; }
}

static void printHelp(void)
{
    printf("--- Commands ---\n");
    printf("  sp N     - set setpoint (0-%d)\n", SP_MAX);
    printf("  hyst N   - set hysteresis (%d-%d)\n", HYST_MIN, HYST_MAX);
    printf("  status   - print current state\n");
    printf("  report   - full report\n");
    printf("  help     - show this list\n");
}

static void printStatus(void)
{
    xSemaphoreTake(xSpMutex, portMAX_DELAY);
    int16_t sp   = spData.setPoint;
    int16_t hyst = spData.hysteresis;
    xSemaphoreGive(xSpMutex);

    xSemaphoreTake(xAcqMutex, portMAX_DELAY);
    int16_t pos = acqData.filteredPosition;
    xSemaphoreGive(xAcqMutex);

    xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
    CtrlAction_t act = ctrlData.action;
    xSemaphoreGive(xCtrlMutex);

    const char *actStr = (act == CTRL_FWD) ? "FWD" :
                         (act == CTRL_REV) ? "REV" : "STOP";
    printf("[STATUS] SP=%d  Pos=%d  Hyst=%d  Motor=%s\n", sp, pos, hyst, actStr);
}

static void taskSerialInput(void *pvParameters)
{
    (void)pvParameters;
    char token[8];

    for (;;)
    {
        if (scanf("%7s", token) != 1)
            continue;

        toLower(token);

        if (strcmp(token, "help") == 0)   { printHelp();   continue; }
        if (strcmp(token, "status") == 0) { printStatus(); continue; }
        if (strcmp(token, "report") == 0) { reportRequested = true; continue; }

        if (strcmp(token, "sp") == 0)
        {
            int val = 0;
            if (scanf("%d", &val) == 1 && val >= SP_MIN && val <= SP_MAX)
            {
                xSemaphoreTake(xSpMutex, portMAX_DELAY);
                spData.setPoint = (int16_t)val;
                xSemaphoreGive(xSpMutex);
                printf("[SP] setpoint -> %d\n", val);
            }
            else
            {
                printf("[ERR] Usage: sp <%d-%d>\n", SP_MIN, SP_MAX);
            }
            continue;
        }

        if (strcmp(token, "hyst") == 0)
        {
            int val = 0;
            if (scanf("%d", &val) == 1 && val >= HYST_MIN && val <= HYST_MAX)
            {
                xSemaphoreTake(xSpMutex, portMAX_DELAY);
                spData.hysteresis = (int16_t)val;
                xSemaphoreGive(xSpMutex);
                printf("[SP] hysteresis -> %d\n", val);
            }
            else
            {
                printf("[ERR] Usage: hyst <%d-%d>\n", HYST_MIN, HYST_MAX);
            }
            continue;
        }

        printf("[ERR] Unknown command '%s'. Type 'help'.\n", token);
    }
}


// Task 2 -- Button input (periodic 50 ms, priority 2)
//
// Polls UP/DOWN buttons with hardware debounce from dd_button.
// Each press adjusts the setpoint by SP_BTN_STEP, clamped to [SP_MIN, SP_MAX].


static void taskButtonInput(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        ddButtonUpdate(BTN_UP_ID);
        ddButtonUpdate(BTN_DOWN_ID);

        bool upPressed   = ddButtonPressed(BTN_UP_ID);
        bool downPressed = ddButtonPressed(BTN_DOWN_ID);

        if (upPressed || downPressed)
        {
            xSemaphoreTake(xSpMutex, portMAX_DELAY);
            int16_t sp = spData.setPoint;

            if (upPressed)
            {
                sp += SP_BTN_STEP;
                if (sp > SP_MAX) sp = SP_MAX;
            }
            if (downPressed)
            {
                sp -= SP_BTN_STEP;
                if (sp < SP_MIN) sp = SP_MIN;
            }

            spData.setPoint = sp;
            xSemaphoreGive(xSpMutex);

            printf("[BTN] setpoint -> %d\n", sp);
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_BUTTON));
    }
}


// Task 3 -- Acquisition (periodic 50 ms, priority 2)
//
// Reads the potentiometer via analogRead, pushes the sample into
// a sliding median filter (MEDIAN_BUF_SIZE samples), and publishes
// both raw and filtered values into AcqData_t.


static void sortBuf(int16_t *arr, uint8_t len)
{
    for (uint8_t i = 1; i < len; i++)
    {
        int16_t val = arr[i];
        int8_t  p   = (int8_t)i - 1;
        while (p >= 0 && arr[p] > val) { arr[p + 1] = arr[p]; p--; }
        arr[p + 1] = val;
    }
}

static int16_t pickMedian(int16_t *samples)
{
    int16_t sorted[MEDIAN_BUF_SIZE];
    memcpy(sorted, samples, sizeof(sorted));
    sortBuf(sorted, MEDIAN_BUF_SIZE);
    return sorted[MEDIAN_BUF_SIZE / 2];
}

static void taskAcquisition(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    int16_t sampleRing[MEDIAN_BUF_SIZE] = {0};
    uint8_t ringPos = 0;

    for (;;)
    {
        int16_t raw = (int16_t)analogRead(PIN_POT);

        sampleRing[ringPos] = raw;
        ringPos = (ringPos + 1) % MEDIAN_BUF_SIZE;

        int16_t filtered = pickMedian(sampleRing);

        xSemaphoreTake(xAcqMutex, portMAX_DELAY);
        acqData.rawPosition      = raw;
        acqData.filteredPosition = filtered;
        xSemaphoreGive(xAcqMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_ACQ));
    }
}


// Task 4 -- ON-OFF Controller (periodic 100 ms, priority 3)
//
// Reads the filtered position and setpoint, computes the error,
// and applies hysteresis-based bang-bang control:
//   error > +hyst  →  motor forward  at MOTOR_FIXED_PCT
//   error < -hyst  →  motor reverse  at MOTOR_FIXED_PCT
//   |error| <= hyst →  motor stop    (dead band)
// The motor is driven directly from this task.


static void taskControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    CtrlAction_t prevAction = CTRL_STOP;

    for (;;)
    {
        xSemaphoreTake(xSpMutex, portMAX_DELAY);
        int16_t sp   = spData.setPoint;
        int16_t hyst = spData.hysteresis;
        xSemaphoreGive(xSpMutex);

        xSemaphoreTake(xAcqMutex, portMAX_DELAY);
        int16_t pos = acqData.filteredPosition;
        xSemaphoreGive(xAcqMutex);

        int16_t error = sp - pos;

        CtrlAction_t action = prevAction;

        if (error > hyst)
            action = CTRL_FWD;
        else if (error < -hyst)
            action = CTRL_REV;
        else if (prevAction != CTRL_STOP)
        {
            // Inside dead band: stop only if we were previously moving
            // and have now entered the band (standard ON-OFF hysteresis)
            if ((prevAction == CTRL_FWD && error <= 0) ||
                (prevAction == CTRL_REV && error >= 0))
                action = CTRL_STOP;
        }

        // Drive motor
        if (action == CTRL_FWD)
            ddMotorSetSpeed(MOTOR_ID, MOTOR_FIXED_PCT, true);
        else if (action == CTRL_REV)
            ddMotorSetSpeed(MOTOR_ID, MOTOR_FIXED_PCT, false);
        else
            ddMotorStop(MOTOR_ID);

        // Track direction changes
        uint32_t now = (uint32_t)millis();
        uint16_t switchInc = (action != prevAction) ? 1 : 0;

        xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
        ctrlData.action  = action;
        ctrlData.error   = error;
        if (action != prevAction)
            ctrlData.lastChangeMs = now;
        ctrlData.switchCount += switchInc;
        xSemaphoreGive(xCtrlMutex);

        prevAction = action;

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONTROL));
    }
}


// Task 5 -- Display & Reporting (priority 1, 500 ms)
//
// LCD rotates through 2 pages every LCD_PAGE_CYCLES cycles:
//   Page 0: SP=nnn  Pos=nnn  /  Err=nnn  Mot:FWD|REV|STOP
//   Page 1: Hyst=nn  SW=nn   /  Age: Ns


static void taskDisplay(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t cycleCount  = 0;
    uint8_t currentPage = 0;

    for (;;)
    {
        SetPointData_t snapSP;
        AcqData_t      snapAcq;
        ControlData_t  snapCtrl;

        xSemaphoreTake(xSpMutex,   portMAX_DELAY); snapSP   = spData;   xSemaphoreGive(xSpMutex);
        xSemaphoreTake(xAcqMutex,  portMAX_DELAY); snapAcq  = acqData;  xSemaphoreGive(xAcqMutex);
        xSemaphoreTake(xCtrlMutex, portMAX_DELAY); snapCtrl = ctrlData; xSemaphoreGive(xCtrlMutex);

        const char *actStr = (snapCtrl.action == CTRL_FWD) ? "FWD" :
                             (snapCtrl.action == CTRL_REV) ? "REV" : "STP";

        // LCD page rotation
        cycleCount++;
        if (cycleCount >= LCD_PAGE_CYCLES)
        {
            cycleCount  = 0;
            currentPage = (uint8_t)((currentPage + 1) % 2);
        }

        char row0[17];
        char row1[17];

        if (currentPage == 0)
        {
            snprintf(row0, sizeof(row0), "SP:%-4d Pos:%-4d",
                     snapSP.setPoint, snapAcq.filteredPosition);
            snprintf(row1, sizeof(row1), "Err:%-4d M:%-3s",
                     snapCtrl.error, actStr);
        }
        else
        {
            uint32_t ageSec = ((uint32_t)millis() - snapCtrl.lastChangeMs) / 1000UL;
            snprintf(row0, sizeof(row0), "Hyst:%-3d SW:%-4u",
                     snapSP.hysteresis, snapCtrl.switchCount);
            snprintf(row1, sizeof(row1), "Age:%-4lus R:%-4d",
                     (unsigned long)ageSec, snapAcq.rawPosition);
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(row0);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(row1);

        // On-demand detailed report
        if (reportRequested)
        {
            reportRequested = false;
            printf("--- Position Control Report ---\n");
            printf("SetPoint     : %d\n",  snapSP.setPoint);
            printf("Hysteresis   : %d\n",  snapSP.hysteresis);
            printf("Raw position : %d\n",  snapAcq.rawPosition);
            printf("Filtered pos : %d\n",  snapAcq.filteredPosition);
            printf("Error        : %d\n",  snapCtrl.error);
            printf("Motor        : %s\n",  actStr);
            printf("Motor speed  : %d%%\n", MOTOR_FIXED_PCT);
            printf("Dir switches : %u\n",  snapCtrl.switchCount);
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_DISPLAY));
    }
}


// Task 6 -- Heartbeat (priority 1, 500 ms)


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


// Initialise all peripherals: serial, motor, buttons, LCD, heartbeat LED.
static void initPeripherals(void)
{
    srvSerialSetup();
    ddMotorSetup(MOTOR_ID, PIN_MOTOR_EN, PIN_MOTOR_IN1, PIN_MOTOR_IN2);
    ddButtonSetup(BTN_UP_ID,   PIN_BTN_UP);
    ddButtonSetup(BTN_DOWN_ID, PIN_BTN_DOWN);
    srvHeartbeatSetup(LED_HB_ID, PIN_LED_HEARTBEAT);
    ddLcdSetup(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
}

// Allocate mutexes for shared data protection between tasks.
static void initMutexes(void)
{
    xSpMutex   = xSemaphoreCreateMutex();
    xAcqMutex  = xSemaphoreCreateMutex();
    xCtrlMutex = xSemaphoreCreateMutex();

    if (!xSpMutex || !xAcqMutex || !xCtrlMutex)
    {
        printf("ERR: mutex creation failed\n");
        for (;;) { }
    }
}

// Create all six FreeRTOS tasks. Halts if any allocation fails.
static void spawnTasks(void)
{
    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(taskSerialInput,  "SerIn",   256, NULL, 1, NULL);
    ok &= xTaskCreate(taskButtonInput,  "BtnIn",   192, NULL, 2, NULL);
    ok &= xTaskCreate(taskAcquisition,  "Acq",     256, NULL, 2, NULL);
    ok &= xTaskCreate(taskControl,      "Ctrl",    256, NULL, 3, NULL);
    ok &= xTaskCreate(taskDisplay,      "Display", 384, NULL, 1, NULL);
    ok &= xTaskCreate(taskHeartbeat,    "HB",      192, NULL, 1, NULL);

    if (ok != pdPASS)
    {
        printf("ERR: task creation failed\n");
        for (;;) { }
    }
}

static void printBanner(void)
{
    printf("=== Lab 5.1  ON-OFF Position Control (Variant B) ===\n");
    printf("Motor  : L298N  EN=pin%d  IN1=pin%d  IN2=pin%d  speed=%d%%\n",
           PIN_MOTOR_EN, PIN_MOTOR_IN1, PIN_MOTOR_IN2, MOTOR_FIXED_PCT);
    printf("Sensor : potentiometer on A0\n");
    printf("Buttons: UP=pin%d  DOWN=pin%d  (step=%d)\n",
           PIN_BTN_UP, PIN_BTN_DOWN, SP_BTN_STEP);
    printf("Default: SP=%d  Hyst=%d\n", SP_DEFAULT, HYST_DEFAULT);
    printf("Timing : btn %dms | acq %dms | ctrl %dms | lcd %dms\n",
           PERIOD_BUTTON, PERIOD_ACQ, PERIOD_CONTROL, PERIOD_DISPLAY);
    printf("Send 'help' for command list.\n");
}

void appLab51Setup()
{
    initPeripherals();
    initMutexes();
    spawnTasks();
    printBanner();
    vTaskStartScheduler();
}

void appLab51Loop()
{
    // FreeRTOS scheduler is running; never reached.
}

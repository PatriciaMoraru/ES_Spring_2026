// Lab 5.2 - PID Light Control
//
// Sensor   : LDR in voltage divider with 10 kOhm -> A0
// Actuator : LED (+220 Ohm) on PIN_LED_PWM (PWM output)
// Display  : I2C LCD 1602 + Serial Plotter
// Input    : Serial STDIO - set <sp> / kp <v> / ki <v> / kd <v> / reset / status / help
//
// Physical setup: LED and LDR are placed at opposite ends of a dark tube
// (e.g. a bubble-tea tube wrapped in insulation tape), forming a closed
// optical feedback loop. The PID controller adjusts the LED brightness
// (0..255 PWM) so the measured light level reaches the Set Point.
//
// FreeRTOS tasks:
//   1. taskSensorRead  - reads LDR every 100 ms, writes SensorData_t       (100 ms,  prio 2)
//   2. taskControl     - PID computation, drives LED PWM                   (100 ms,  prio 3)
//   3. taskDisplay     - LCD pages + Serial Plotter line                   (500 ms,  prio 1)
//   4. taskCmdRead     - serial command parser, adjusts SP and gains       (event,   prio 1)
//   5. taskHeartbeat   - heartbeat blink                                   (500 ms,  prio 1)

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "app_lab_5_2.h"
#include "dd_ldr/dd_ldr.h"
#include "dd_led_pwm/dd_led_pwm.h"
#include "dd_lcd/dd_lcd.h"
#include "dd_led/dd_led.h"
#include "ctrl_pid/ctrl_pid.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"


// -- Pin assignments (D2, D3, D5 reserved by FreeRTOS Timer 3) ----------

#define PIN_LDR           A0
#define PIN_LED_PWM       9        // PWM on Timer 2, safe with FreeRTOS
#define PIN_LED_HEARTBEAT 13

#define LCD_I2C_ADDR  0x27
#define LCD_COLS      16
#define LCD_ROWS      2

#define LED_ACT       0
#define LED_HEARTBEAT 0

#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)


// -- Task periods (ms) --------------------------------------------------

// The LDR has significant response lag (typ. 100-300 ms from bright -> dark).
// Running the control loop at 200 ms keeps the PID from fighting its own
// delayed echo, which is especially important at low set points where the
// LDR's logarithmic response makes plant gain higher.
#define PERIOD_SENSOR    200
#define PERIOD_CONTROL   200
#define PERIOD_DISPLAY   500
#define PERIOD_HEARTBEAT 500

#define CONTROL_DT_S  (PERIOD_CONTROL / 1000.0f)

// Exponential moving average weight for the light measurement
// filteredLight = LIGHT_EMA_ALPHA * raw + (1 - LIGHT_EMA_ALPHA) * filteredLight
// 0.35 = responsive but noticeably smoother than raw.
#define LIGHT_EMA_ALPHA  0.35f


// -- Default control parameters -----------------------------------------

#define DEFAULT_SET_POINT  50.0f   // target light level in %

// Tuning notes: the optical loop has almost no delay (LED -> LDR is
// effectively instantaneous) so the loop is very gain-sensitive. Empirical
// tests showed Ku ~1.5-2 (the system oscillates at Kp=2.0). These values
// are ~25-30% of Ku with a small Ki for offset removal and a light Kd to
// dampen approach without derivative kick from sensor jitter.
#define DEFAULT_KP         0.5f
#define DEFAULT_KI         0.1f
#define DEFAULT_KD         0.02f

#define PID_OUT_MIN        0.0f
#define PID_OUT_MAX        255.0f

#define LCD_PAGE_CYCLES    3


// -- Shared data types --------------------------------------------------

typedef struct
{
    int   raw;        // raw ADC 0..1023
    float percent;    // 0..100
    bool  valid;
} SensorData_t;

typedef struct
{
    float setPoint;   // 0..100
    float kp;
    float ki;
    float kd;
    float output;     // last PID output (0..255)
} ControlData_t;


// -- Shared instances + mutexes -----------------------------------------

static SensorData_t  sensorData  = {0, 0.0f, false};
static ControlData_t controlData = {DEFAULT_SET_POINT, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 0.0f};

static CtrlPid_t     pid;

static SemaphoreHandle_t xSensorMutex = NULL;
static SemaphoreHandle_t xCtrlMutex   = NULL;


// -- Task 1 - Sensor Acquisition (200 ms, priority 2) ------------------
//
// Reads the LDR and stores the raw ADC value plus a 0..100 % light level
// (0 = dark, 100 = bright). The KY-018 wiring inversion is handled inside
// the dd_ldr driver (see dd_ldr.h).
//
// An exponential moving average smooths over the LDR's own response lag
// (LDRs take 100-300 ms to drop from bright to dark), which would
// otherwise trick the PID into fighting a delayed echo of its own output.

static void taskSensorRead(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    float filtered = 0.0f;
    bool  first    = true;

    for (;;)
    {
        int   raw       = ddLdrRead();
        float sample    = ddLdrGetLight();

        if (first)
        {
            filtered = sample;
            first    = false;
        }
        else
        {
            filtered = LIGHT_EMA_ALPHA * sample
                     + (1.0f - LIGHT_EMA_ALPHA) * filtered;
        }

        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        sensorData.raw     = raw;
        sensorData.percent = filtered;
        sensorData.valid   = true;
        xSemaphoreGive(xSensorMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_SENSOR));
    }
}


// -- Task 2 - PID Control (100 ms, priority 3) -------------------------
//
// Snapshots the latest sensor reading and the current gains, runs the
// discrete PID to compute a PWM duty in [0, 255], and writes it to the
// LED actuator. The resulting brightness changes what the LDR sees,
// closing the optical feedback loop.

static void taskControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        SensorData_t snap;
        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        snap = sensorData;
        xSemaphoreGive(xSensorMutex);

        xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
        float sp = controlData.setPoint;
        ctrlPidSetGains(&pid, controlData.kp, controlData.ki, controlData.kd);
        xSemaphoreGive(xCtrlMutex);

        float output = 0.0f;
        if (snap.valid)
            output = ctrlPidCompute(&pid, sp, snap.percent);

        uint8_t duty = (uint8_t)lroundf(output);
        ddLedPwmWrite(LED_ACT, duty);

        xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
        controlData.output = output;
        xSemaphoreGive(xCtrlMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONTROL));
    }
}


// -- Task 3 - Display & Serial Plotter (500 ms, priority 1) ------------
//
// LCD rotates between two pages every LCD_PAGE_CYCLES cycles.
//   Page 0:  L:XX% SP:YY%    /   OUT:ZZZ Kp:N.NN
//   Page 1:  Ki:X.XX Kd:Y.YY /   raw:RRRR
//
// Each cycle also emits one Serial Plotter line so the three traces
// (SetPoint, Light, Output) share the same time axis in the plotter.
// Output is scaled from [0..255] to [0..100] so all three series fit on
// a comparable Y-axis.

static void taskDisplay(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t cycleCount  = 0;
    uint8_t currentPage = 0;

    for (;;)
    {
        SensorData_t snapS;
        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        snapS = sensorData;
        xSemaphoreGive(xSensorMutex);

        ControlData_t snapC;
        xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
        snapC = controlData;
        xSemaphoreGive(xCtrlMutex);

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
            if (snapS.valid)
                snprintf(row0, sizeof(row0), "L:%3d%% SP:%3d%%",
                         (int)lroundf(snapS.percent),
                         (int)lroundf(snapC.setPoint));
            else
                snprintf(row0, sizeof(row0), "L:--%%  SP:%3d%%",
                         (int)lroundf(snapC.setPoint));

            snprintf(row1, sizeof(row1), "OUT:%3d Kp:%.2f",
                     (int)lroundf(snapC.output), (double)snapC.kp);
        }
        else
        {
            snprintf(row0, sizeof(row0), "Ki:%.2f Kd:%.2f",
                     (double)snapC.ki, (double)snapC.kd);

            if (snapS.valid)
                snprintf(row1, sizeof(row1), "raw:%4d", snapS.raw);
            else
                snprintf(row1, sizeof(row1), "raw:----");
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(row0);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(row1);

        // Teleplot output — one variable per line, format: >name:value|g:group
        // All three traces share the group "control" so Teleplot displays
        // them overlaid on a single plot by default (no manual drag/drop).
        // Output is scaled from [0..255] to [0..100] so all three fit on
        // the same Y-axis range.
        float outPlot   = snapC.output * (100.0f / 255.0f);
        float lightPlot = snapS.valid ? snapS.percent : 0.0f;

        printf(">SetPoint:%.2f|g:control\n", (double)snapC.setPoint);
        printf(">Light:%.2f|g:control\n",    (double)lightPlot);
        printf(">Output:%.2f|g:control\n",   (double)outPlot);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_DISPLAY));
    }
}


// -- Task 4 - Command Reader (event-driven, priority 1) ----------------
//
// Blocks on scanf() for serial tokens. Yields via the vTaskDelay(1)
// inside srvSerialGetChar (SERIAL_FREERTOS_YIELD flag).
//
// Commands:
//   set <0-100>   - change Set Point (%)
//   kp <value>    - set proportional gain
//   ki <value>    - set integral gain
//   kd <value>    - set derivative gain
//   reset         - clear integral and derivative memory
//   status        - print current state
//   help          - list commands

static void taskCmdRead(void *pvParameters)
{
    (void)pvParameters;

    char cmd[8];

    for (;;)
    {
        if (scanf("%7s", cmd) != 1)
            continue;

        for (uint8_t i = 0; cmd[i]; i++)
            if (cmd[i] >= 'A' && cmd[i] <= 'Z')
                cmd[i] += 32;

        if (strcmp(cmd, "help") == 0)
        {
            printf("Commands:\n");
            printf("  set <0-100> - set target light level (%%)\n");
            printf("  kp <value>  - set proportional gain\n");
            printf("  ki <value>  - set integral gain\n");
            printf("  kd <value>  - set derivative gain\n");
            printf("  reset       - clear PID integral/derivative\n");
            printf("  status      - print current state\n");
            printf("  help        - show this list\n");
            continue;
        }

        if (strcmp(cmd, "status") == 0)
        {
            SensorData_t snapS;
            xSemaphoreTake(xSensorMutex, portMAX_DELAY);
            snapS = sensorData;
            xSemaphoreGive(xSensorMutex);

            ControlData_t snapC;
            xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
            snapC = controlData;
            xSemaphoreGive(xCtrlMutex);

            printf("--- Status ---\n");
            if (snapS.valid)
                printf("Light     : %.2f %% (raw %d)\n",
                       (double)snapS.percent, snapS.raw);
            else
                printf("Light     : invalid read\n");
            printf("Set Point : %.2f %%\n", (double)snapC.setPoint);
            printf("Kp        : %.3f\n", (double)snapC.kp);
            printf("Ki        : %.3f\n", (double)snapC.ki);
            printf("Kd        : %.3f\n", (double)snapC.kd);
            printf("Integral  : %.3f\n", (double)pid.integral);
            printf("Output    : %.2f (PWM 0-255)\n", (double)snapC.output);
            continue;
        }

        if (strcmp(cmd, "reset") == 0)
        {
            xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
            ctrlPidReset(&pid);
            xSemaphoreGive(xCtrlMutex);
            printf("[CTRL] PID state cleared\n");
            continue;
        }

        if (strcmp(cmd, "set") == 0)
        {
            float val = 0.0f;
            if (scanf("%f", &val) == 1 && val >= 0.0f && val <= 100.0f)
            {
                xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
                controlData.setPoint = val;
                xSemaphoreGive(xCtrlMutex);
                printf("[CTRL] Set Point = %.2f %%\n", (double)val);
            }
            else
            {
                printf("[ERR] Usage: set <level>  (0 - 100)\n");
            }
            continue;
        }

        if (strcmp(cmd, "kp") == 0 || strcmp(cmd, "ki") == 0 || strcmp(cmd, "kd") == 0)
        {
            float val = 0.0f;
            if (scanf("%f", &val) == 1 && val >= 0.0f && val < 1000.0f)
            {
                xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
                if      (cmd[1] == 'p') controlData.kp = val;
                else if (cmd[1] == 'i') controlData.ki = val;
                else                    controlData.kd = val;
                ctrlPidReset(&pid);
                xSemaphoreGive(xCtrlMutex);
                printf("[CTRL] %c%c = %.3f (PID reset)\n",
                       cmd[0], cmd[1], (double)val);
            }
            else
            {
                printf("[ERR] Usage: %s <value>  (0 - 1000)\n", cmd);
            }
            continue;
        }

        printf("[ERR] Unknown command '%s'. Type 'help'.\n", cmd);
    }
}


// -- Task 5 - Heartbeat (500 ms, priority 1) ---------------------------

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


// -- Setup & Loop ------------------------------------------------------

void appLab52Setup()
{
    srvSerialSetup();

    ddLdrSetup(PIN_LDR);
    ddLedPwmSetup(LED_ACT, PIN_LED_PWM);

    srvHeartbeatSetup(LED_HEARTBEAT, PIN_LED_HEARTBEAT);

    ddLcdSetup(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

    ctrlPidInit(&pid,
                DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
                PID_OUT_MIN, PID_OUT_MAX,
                CONTROL_DT_S);

    xSensorMutex = xSemaphoreCreateMutex();
    xCtrlMutex   = xSemaphoreCreateMutex();

    if (!xSensorMutex || !xCtrlMutex)
    {
        printf("FATAL: mutex creation failed\n");
        for (;;) { }
    }

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(taskSensorRead, "Sensor",  192, NULL, 2, NULL);
    ok &= xTaskCreate(taskControl,    "Control", 256, NULL, 3, NULL);
    ok &= xTaskCreate(taskDisplay,    "Display", 384, NULL, 1, NULL);
    ok &= xTaskCreate(taskCmdRead,    "CmdRead", 256, NULL, 1, NULL);
    ok &= xTaskCreate(taskHeartbeat,  "HB",      192, NULL, 1, NULL);

    if (ok != pdPASS)
    {
        printf("FATAL: task creation failed\n");
        for (;;) { }
    }

    printf("Lab 5.2 - PID Light Control\n");
    printf("Sensor : LDR on A%d (raw 0-1023 -> 0-100%%)\n", PIN_LDR - A0);
    printf("LED    : PWM pin %d (0-255)\n", PIN_LED_PWM);
    printf("SP=%.1f%%  Kp=%.2f Ki=%.2f Kd=%.2f  dt=%.2fs\n",
           (double)DEFAULT_SET_POINT,
           (double)DEFAULT_KP, (double)DEFAULT_KI, (double)DEFAULT_KD,
           (double)CONTROL_DT_S);
    printf("Type 'help' for commands.\n");

    vTaskStartScheduler();
}

void appLab52Loop()
{
    // FreeRTOS scheduler is running; this is never reached.
}

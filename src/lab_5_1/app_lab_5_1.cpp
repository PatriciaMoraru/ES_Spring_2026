// Lab 5.1 - ON-OFF Temperature Control with Hysteresis (Variant A)
//
// Sensor   : DHT22 on PIN_DHT
// Actuator : 5V relay module on PIN_RELAY (active-LOW) — switches a 12V bulb
// Display  : I2C LCD 1602 + Serial Plotter
// Input    : Serial STDIO — set <temp> / hyst <band> / status / help
//
// FreeRTOS tasks:
//   1. taskSensorRead  - reads DHT every 2 s, writes SensorData_t      (2000 ms, prio 2)
//   2. taskControl     - ON-OFF hysteresis, drives relay                ( 500 ms, prio 3)
//   3. taskDisplay     - LCD pages + Serial Plotter line                (1000 ms, prio 1)
//   4. taskCmdRead     - serial command parser, adjusts setpoint/hyst   (event-driven, prio 1)
//   5. taskHeartbeat   - heartbeat blink                                ( 500 ms, prio 1)

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_5_1.h"
#include "dd_dht/dd_dht.h"
#include "dd_actuator/dd_actuator.h"
#include "dd_lcd/dd_lcd.h"
#include "dd_led/dd_led.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"

// ── Pin assignments (D2, D3, D5 reserved by FreeRTOS Timer 3) ──────────

#define PIN_DHT 4
#define PIN_RELAY 7
#define PIN_LED_HEARTBEAT 13

#define LCD_I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

#define ACT_RELAY 0
#define LED_HEARTBEAT 0

#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)

// ── Task periods (ms) ──────────────────────────────────────────────────

#define PERIOD_SENSOR 2000
#define PERIOD_CONTROL 500
#define PERIOD_DISPLAY 1000
#define PERIOD_HEARTBEAT 500

// ── Default control parameters ─────────────────────────────────────────

#define DEFAULT_SET_POINT 29.0f
#define DEFAULT_HYSTERESIS 1.0f

// LCD page rotation: switch every N display cycles
#define LCD_PAGE_CYCLES 3

// ── Shared data types ──────────────────────────────────────────────────

typedef struct
{
    float temperature;
    float humidity;
    bool valid;
} SensorData_t;

typedef struct
{
    float setPoint;
    float hysteresis;
    bool relayState;
} ControlData_t;

// ── Shared instances + mutexes ─────────────────────────────────────────

static SensorData_t sensorData = {0.0f, 0.0f, false};
static ControlData_t controlData = {DEFAULT_SET_POINT, DEFAULT_HYSTERESIS, false};

static SemaphoreHandle_t xSensorMutex = NULL;
static SemaphoreHandle_t xCtrlMutex = NULL;

// ── Task 1 — Sensor Acquisition (2000 ms, priority 2) ─────────────────
//
// Reads the DHT sensor and stores the latest temperature and humidity.
// The 2 s period satisfies the DHT11 minimum sampling interval;
// DHT22 is also comfortable at this rate.

static void taskSensorRead(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        bool ok = ddDhtRead();

        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        if (ok)
        {
            sensorData.temperature = ddDhtGetTemperature();
            sensorData.humidity = ddDhtGetHumidity();
            sensorData.valid = true;
        }
        else
        {
            sensorData.valid = false;
        }
        xSemaphoreGive(xSensorMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_SENSOR));
    }
}

// ── Task 2 — ON-OFF Control with Hysteresis (500 ms, priority 3) ──────
//
// Reads the latest sensor snapshot and the current setpoint/hysteresis,
// then applies the ON-OFF law:
//   temp < (SP - hyst)  →  relay ON  (heat)
//   temp > (SP + hyst)  →  relay OFF
//   otherwise           →  keep previous state

static void taskControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        // Read sensor snapshot
        SensorData_t snap;
        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        snap = sensorData;
        xSemaphoreGive(xSensorMutex);

        // Read current control parameters
        xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
        float sp = controlData.setPoint;
        float hyst = controlData.hysteresis;
        bool prev = controlData.relayState;
        xSemaphoreGive(xCtrlMutex);

        bool newState = prev;

        if (snap.valid)
        {
            float lo = sp - hyst;
            float hi = sp + hyst;

            if (snap.temperature < lo)
                newState = true;
            else if (snap.temperature > hi)
                newState = false;
            // else: keep prev
        }

        // Drive actuator
        if (newState)
            ddActuatorOn(ACT_RELAY);
        else
            ddActuatorOff(ACT_RELAY);

        // Write back relay state
        xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
        controlData.relayState = newState;
        xSemaphoreGive(xCtrlMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONTROL));
    }
}

// ── Task 3 — Display & Serial Plotter (1000 ms, priority 1) ───────────
//
// LCD alternates between two pages every LCD_PAGE_CYCLES cycles.
//   Page 0:  T:<temp>C SP:<sp>C   /   Relay:ON|OFF H:+/-<h>
//   Page 1:  Lo:<lo>  Hi:<hi>     /   Hum:<hum>%
//
// Each cycle also emits one Serial Plotter line so the three traces
// (SetPoint, Temperature, Relay) are visible on the same Y-axis.

static void taskDisplay(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t cycleCount = 0;
    uint8_t currentPage = 0;

    for (;;)
    {
        // Snapshots
        SensorData_t snapS;
        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        snapS = sensorData;
        xSemaphoreGive(xSensorMutex);

        ControlData_t snapC;
        xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
        snapC = controlData;
        xSemaphoreGive(xCtrlMutex);

        float lo = snapC.setPoint - snapC.hysteresis;
        float hi = snapC.setPoint + snapC.hysteresis;

        // ── LCD page rotation ──
        cycleCount++;
        if (cycleCount >= LCD_PAGE_CYCLES)
        {
            cycleCount = 0;
            currentPage = (currentPage + 1) % 2;
        }

        char row0[17];
        char row1[17];

        if (currentPage == 0)
        {
            if (snapS.valid)
                snprintf(row0, sizeof(row0), "T:%.1fC SP:%.1fC", (double)snapS.temperature, (double)snapC.setPoint);
            else
                snprintf(row0, sizeof(row0), "T:--.- SP:%.1fC", (double)snapC.setPoint);

            snprintf(row1, sizeof(row1), "Relay:%-3s H:%.1f",
                     snapC.relayState ? "ON" : "OFF", (double)snapC.hysteresis);
        }
        else
        {
            snprintf(row0, sizeof(row0), "Lo:%.1f Hi:%.1f", (double)lo, (double)hi);

            if (snapS.valid)
                snprintf(row1, sizeof(row1), "Hum:%.1f%%", (double)snapS.humidity);
            else
                snprintf(row1, sizeof(row1), "Hum:--.-%%");
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(row0);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(row1);

        // ── Teleplot output ──
        // Five traces on the same graph showing the hysteresis band:
        //   Temperature  — actual sensor reading
        //   SetPoint     — reference value
        //   Lo           — lower threshold (SP - hyst): relay turns ON below this
        //   Hi           — upper threshold (SP + hyst): relay turns OFF above this
        //   Relay        — scaled to SP so it is visible on the same axis (0 = OFF)
        float relayPlot = snapC.relayState ? snapC.setPoint : 0.0f;
        float temp = snapS.valid ? snapS.temperature : 0.0f;

        // Teleplot format — drag all 5 from the sidebar onto one chart.
        printf(">Temperature:%.2f\n", (double)temp);
        printf(">SetPoint:%.2f\n",    (double)snapC.setPoint);
        printf(">Lo:%.2f\n",          (double)lo);
        printf(">Hi:%.2f\n",          (double)hi);
        printf(">Relay:%.2f\n",       (double)relayPlot);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_DISPLAY));
    }
}

// ── Task 4 — Command Reader (event-driven, priority 1) ────────────────
//
// Blocks on scanf() waiting for serial tokens.  Yields via the
// vTaskDelay(1) inside srvSerialGetChar (SERIAL_FREERTOS_YIELD flag).
//
// Commands:
//   set <value>   — change Set Point (°C)
//   hyst <value>  — change hysteresis band (°C)
//   status        — print current state
//   help          — list commands

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
            printf("  set <temp>  - set target temperature (C)\n");
            printf("  hyst <val>  - set hysteresis band (C)\n");
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
                printf("Temperature : %.2f C\n", (double)snapS.temperature);
            else
                printf("Temperature : invalid read\n");
            printf("Humidity    : %.2f %%\n", (double)snapS.humidity);
            printf("Set Point   : %.2f C\n", (double)snapC.setPoint);
            printf("Hysteresis  : %.2f C\n", (double)snapC.hysteresis);
            printf("Thresholds  : Lo=%.2f  Hi=%.2f\n",
                   (double)(snapC.setPoint - snapC.hysteresis),
                   (double)(snapC.setPoint + snapC.hysteresis));
            printf("Relay       : %s\n", snapC.relayState ? "ON" : "OFF");
            continue;
        }

        if (strcmp(cmd, "set") == 0)
        {
            float val = 0.0f;
            if (scanf("%f", &val) == 1 && val > 0.0f && val < 80.0f)
            {
                xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
                controlData.setPoint = val;
                xSemaphoreGive(xCtrlMutex);
                printf("[CTRL] Set Point = %.2f C\n", (double)val);
            }
            else
            {
                printf("[ERR] Usage: set <temp>  (0 - 80)\n");
            }
            continue;
        }

        if (strcmp(cmd, "hyst") == 0)
        {
            float val = 0.0f;
            if (scanf("%f", &val) == 1 && val > 0.0f && val < 20.0f)
            {
                xSemaphoreTake(xCtrlMutex, portMAX_DELAY);
                controlData.hysteresis = val;
                xSemaphoreGive(xCtrlMutex);
                printf("[CTRL] Hysteresis = %.2f C\n", (double)val);
            }
            else
            {
                printf("[ERR] Usage: hyst <band>  (0 - 20)\n");
            }
            continue;
        }

        printf("[ERR] Unknown command '%s'. Type 'help'.\n", cmd);
    }
}

// ── Task 5 — Heartbeat (500 ms, priority 1) ───────────────────────────

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

// ── Setup & Loop ───────────────────────────────────────────────────────

void appLab51Setup()
{
    srvSerialSetup();

    ddDhtSetup(PIN_DHT, DD_DHT22);

    ddActuatorSetup(ACT_RELAY, PIN_RELAY, true); // active-LOW relay module

    srvHeartbeatSetup(LED_HEARTBEAT, PIN_LED_HEARTBEAT);

    ddLcdSetup(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

    xSensorMutex = xSemaphoreCreateMutex();
    xCtrlMutex = xSemaphoreCreateMutex();

    if (!xSensorMutex || !xCtrlMutex)
    {
        printf("FATAL: mutex creation failed\n");
        for (;;)
        {
        }
    }

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(taskSensorRead, "Sensor", 256, NULL, 2, NULL);
    ok &= xTaskCreate(taskControl, "Control", 256, NULL, 3, NULL);
    ok &= xTaskCreate(taskDisplay, "Display", 384, NULL, 1, NULL);
    ok &= xTaskCreate(taskCmdRead, "CmdRead", 256, NULL, 1, NULL);
    ok &= xTaskCreate(taskHeartbeat, "HB", 192, NULL, 1, NULL);

    if (ok != pdPASS)
    {
        printf("FATAL: task creation failed\n");
        for (;;)
        {
        }
    }

    printf("Lab 5.1 - ON-OFF Temperature Control with Hysteresis\n");
    printf("Sensor  : DHT22 on pin %d\n", PIN_DHT);
    printf("Relay   : pin %d (active-LOW)\n", PIN_RELAY);
    printf("Set Pt  : %.1f C   Hysteresis: %.1f C\n",
           (double)DEFAULT_SET_POINT, (double)DEFAULT_HYSTERESIS);
    printf("Type 'help' for commands.\n");

    vTaskStartScheduler();
}

void appLab51Loop()
{
    // FreeRTOS scheduler is running; this is never reached.
}

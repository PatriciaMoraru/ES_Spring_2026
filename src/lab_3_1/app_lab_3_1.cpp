// Lab 3.1 - Dual sensor monitoring system (FreeRTOS, Variant C)
//
// Sensors:  LDR (analog, A0)  +  DHT11 (digital, pin 22)
// Display:  LCD 1602 I2C  +  Serial STDIO
// Alerts:   LED green/yellow/red with hysteresis & anti-bounce
//
// FreeRTOS tasks:
//   1. taskSensorAcq   - reads both sensors            (prio 3, 100ms)
//   2. taskCondition    - threshold alerting + LEDs     (prio 2, 200ms)
//   3. taskDisplay      - LCD pages + serial report     (prio 1, 1000ms)
//   4. taskHeartbeat    - heartbeat LED                 (prio 1, 500ms)
//
// Sound sensor (dd_sound) is available but disabled for now.
// To re-enable: add PIN_SOUND, include dd_sound, and wire into tasks.

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>

#include "app_lab_3_1.h"
#include "dd_led/dd_led.h"
#include "dd_lcd/dd_lcd.h"
#include "dd_ldr/dd_ldr.h"
#include "dd_dht/dd_dht.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"

// ── Pin assignments (avoid 2, 3, 5 — used by FreeRTOS Timer 3) ──

#define PIN_LDR A0
#define PIN_DHT 22
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define PIN_LED_GREEN 7
#define PIN_LED_RED 8
#define PIN_LED_YELLOW 9
#define PIN_LED_HEARTBEAT 13

// ── Logical LED IDs ──

#define LED_GREEN 0
#define LED_RED 1
#define LED_YELLOW 2
#define LED_HEARTBEAT 3

// ── Tick helper ──

#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)

// ── Task scheduling (ms) ──

#define PERIOD_ACQUIRE 100
#define PERIOD_CONDITION 200
#define PERIOD_DISPLAY 1000
#define PERIOD_HEARTBEAT 500

// DHT11 physical minimum read interval
#define DHT_READ_INTERVAL_MS 2000

// ── Threshold configuration — Temperature (°C) ──

#define TEMP_WARN_THRESHOLD 28
#define TEMP_ALERT_THRESHOLD 32
#define TEMP_HYSTERESIS 2

// ── Threshold configuration — Light level (0-100 %) ──
// Higher percent = darker = worse (inverted LDR module)

#define LDR_WARN_THRESHOLD 60
#define LDR_ALERT_THRESHOLD 75
#define LDR_HYSTERESIS 5

// Consecutive readings to confirm a state change
#define DEBOUNCE_COUNT 3

// LCD page rotation: switch every N display cycles
#define LCD_PAGE_CYCLES 3

// Statistics window: reset min/max/avg every N display cycles (1 cycle = 1s)
#define STATS_WINDOW_CYCLES 30

// ── Data types ──

typedef enum
{
    LEVEL_NORMAL,
    LEVEL_WARNING,
    LEVEL_ALERT
} AlertLevel_t;

typedef struct
{
    int ldrRaw;
    uint8_t ldrPercent;
    int ldrJitter;
    bool ldrConnected;
    float temperature;
    float humidity;
    bool dhtValid;
} SensorData_t;

typedef struct
{
    AlertLevel_t tempLevel;
    AlertLevel_t ldrLevel;
    AlertLevel_t overallLevel;
    unsigned long tempAlertCount;
    unsigned long ldrAlertCount;
} AlertState_t;

// ── Shared data ──

static SensorData_t sensorData = {0, 0, 0, false, 0.0, 0.0, false};
static AlertState_t alertState = {LEVEL_NORMAL, LEVEL_NORMAL, LEVEL_NORMAL, 0, 0};

// ── Synchronization ──

static SemaphoreHandle_t xSensorMutex = NULL;
static SemaphoreHandle_t xAlertMutex = NULL;

//  Helper functions (used by taskCondition)

static const char *levelStr(AlertLevel_t level)
{
    switch (level)
    {
    case LEVEL_WARNING:
        return "WARN";
    case LEVEL_ALERT:
        return "ALERT";
    default:
        return "OK";
    }
}

// Evaluate temperature alert level with hysteresis.
// Thresholds shift down by TEMP_HYSTERESIS when returning to a lower level.
static AlertLevel_t evalTempLevel(float temp, AlertLevel_t current)
{
    switch (current)
    {
    case LEVEL_NORMAL:
        if (temp >= TEMP_WARN_THRESHOLD)
            return LEVEL_WARNING;
        break;
    case LEVEL_WARNING:
        if (temp >= TEMP_ALERT_THRESHOLD)
            return LEVEL_ALERT;
        if (temp < TEMP_WARN_THRESHOLD - TEMP_HYSTERESIS)
            return LEVEL_NORMAL;
        break;
    case LEVEL_ALERT:
        if (temp < TEMP_ALERT_THRESHOLD - TEMP_HYSTERESIS)
            return LEVEL_WARNING;
        break;
    }
    return current;
}

// Evaluate LDR alert level with hysteresis (inverted module: higher = darker).
// Thresholds shift down by LDR_HYSTERESIS when returning to a lower severity.
static AlertLevel_t evalLdrLevel(uint8_t percent, AlertLevel_t current)
{
    switch (current)
    {
    case LEVEL_NORMAL:
        if (percent >= LDR_WARN_THRESHOLD)
            return LEVEL_WARNING;
        break;
    case LEVEL_WARNING:
        if (percent >= LDR_ALERT_THRESHOLD)
            return LEVEL_ALERT;
        if (percent < LDR_WARN_THRESHOLD - LDR_HYSTERESIS)
            return LEVEL_NORMAL;
        break;
    case LEVEL_ALERT:
        if (percent < LDR_ALERT_THRESHOLD - LDR_HYSTERESIS)
            return LEVEL_WARNING;
        break;
    }
    return current;
}

// Anti-bounce filter: only accept a new level after DEBOUNCE_COUNT
// consecutive evaluations agree on the same candidate.
static AlertLevel_t applyDebounce(AlertLevel_t current,
                                  AlertLevel_t candidate,
                                  AlertLevel_t *pending,
                                  uint8_t *counter)
{
    if (candidate != current)
    {
        if (candidate == *pending)
        {
            (*counter)++;
            if (*counter >= DEBOUNCE_COUNT)
            {
                *counter = 0;
                *pending = candidate;
                return candidate;
            }
        }
        else
        {
            *pending = candidate;
            *counter = 1;
        }
    }
    else
    {
        *counter = 0;
        *pending = current;
    }
    return current;
}

//  Task 1 — Sensor Acquisition  (priority 3, 100 ms)
// Reads LDR every cycle.  Reads DHT11 every DHT_READ_INTERVAL_MS
// (DHT11 needs ≥ 1 s between physical reads).

static void taskSensorAcq(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    unsigned long lastDhtReadMs = 0;
    float cachedTemp = 0.0;
    float cachedHum = 0.0;
    bool cachedValid = false;

    for (;;)
    {
        int raw = ddLdrRead();
        uint8_t pct = ddLdrGetPercent();

        unsigned long now = millis();
        if (now - lastDhtReadMs >= DHT_READ_INTERVAL_MS)
        {
            if (ddDhtRead())
            {
                cachedTemp = ddDhtGetTemperature();
                cachedHum = ddDhtGetHumidity();
                cachedValid = true;
            }
            else
            {
                cachedValid = false;
            }
            lastDhtReadMs = now;
        }

        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        sensorData.ldrRaw = raw;
        sensorData.ldrPercent = pct;
        sensorData.ldrJitter = ddLdrGetJitter();
        sensorData.ldrConnected = ddLdrIsConnected();
        sensorData.temperature = cachedTemp;
        sensorData.humidity = cachedHum;
        sensorData.dhtValid = cachedValid;
        xSemaphoreGive(xSensorMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_ACQUIRE));
    }
}

//  Task 2 — Threshold Conditioning  (priority 2, 200 ms)
// Reads sensor data, applies hysteresis + anti-bounce, drives LEDs.

static void taskCondition(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    AlertLevel_t curTemp = LEVEL_NORMAL;
    AlertLevel_t curLdr = LEVEL_NORMAL;

    AlertLevel_t pendingTemp = LEVEL_NORMAL;
    uint8_t debounceTemp = 0;

    AlertLevel_t pendingLdr = LEVEL_NORMAL;
    uint8_t debounceLdr = 0;

    for (;;)
    {
        SensorData_t snap;
        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        snap = sensorData;
        xSemaphoreGive(xSensorMutex);

        AlertLevel_t candTemp = evalTempLevel(snap.temperature, curTemp);
        AlertLevel_t candLdr = evalLdrLevel(snap.ldrPercent, curLdr);

        AlertLevel_t prevTemp = curTemp;
        AlertLevel_t prevLdr = curLdr;

        curTemp = applyDebounce(curTemp, candTemp, &pendingTemp, &debounceTemp);
        curLdr = applyDebounce(curLdr, candLdr, &pendingLdr, &debounceLdr);

        AlertLevel_t overall = (curTemp > curLdr) ? curTemp : curLdr;

        xSemaphoreTake(xAlertMutex, portMAX_DELAY);
        alertState.tempLevel = curTemp;
        alertState.ldrLevel = curLdr;
        if (curTemp > prevTemp)
            alertState.tempAlertCount++;
        if (curLdr > prevLdr)
            alertState.ldrAlertCount++;

        bool levelChanged = (overall != alertState.overallLevel);
        alertState.overallLevel = overall;
        xSemaphoreGive(xAlertMutex);

        if (levelChanged)
        {
            for (int i = 0; i < 2; i++)
            {
                ddLedOn(LED_GREEN);
                ddLedOn(LED_YELLOW);
                ddLedOn(LED_RED);
                vTaskDelay(MS_TO_TICKS(100));
                ddLedOff(LED_GREEN);
                ddLedOff(LED_YELLOW);
                ddLedOff(LED_RED);
                vTaskDelay(MS_TO_TICKS(100));
            }
        }

        ddLedOff(LED_GREEN);
        ddLedOff(LED_YELLOW);
        ddLedOff(LED_RED);

        switch (overall)
        {
        case LEVEL_NORMAL:
            ddLedOn(LED_GREEN);
            break;
        case LEVEL_WARNING:
            ddLedOn(LED_YELLOW);
            break;
        case LEVEL_ALERT:
            ddLedOn(LED_RED);
            break;
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONDITION));
    }
}

//  Task 3 — Display & Reporting  (priority 1, 1000 ms)
// LCD alternates between two pages every LCD_PAGE_CYCLES seconds.
// Serial report is printed every cycle.

static void taskDisplay(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t cycleCount = 0;
    uint8_t currentPage = 0;

    // ── statistics tracking ──
    int statsTempMin = 9999, statsTempMax = -9999;
    long statsTempSum = 0;
    int statsHumMin = 9999, statsHumMax = -9999;
    long statsHumSum = 0;
    int statsLdrMin = 101, statsLdrMax = -1;
    long statsLdrSum = 0;
    unsigned int statsSampleCount = 0;

    for (;;)
    {
        // ── snapshot shared data ──
        SensorData_t snapS;
        AlertState_t snapA;

        xSemaphoreTake(xSensorMutex, portMAX_DELAY);
        snapS = sensorData;
        xSemaphoreGive(xSensorMutex);

        xSemaphoreTake(xAlertMutex, portMAX_DELAY);
        snapA = alertState;
        xSemaphoreGive(xAlertMutex);

        int tempI = (int)snapS.temperature;
        int humI = (int)snapS.humidity;
        int ldrPct = (int)snapS.ldrPercent;

        // ── update statistics ──
        if (snapS.dhtValid)
        {
            if (tempI < statsTempMin)
                statsTempMin = tempI;
            if (tempI > statsTempMax)
                statsTempMax = tempI;
            statsTempSum += tempI;
            if (humI < statsHumMin)
                statsHumMin = humI;
            if (humI > statsHumMax)
                statsHumMax = humI;
            statsHumSum += humI;
        }
        if (ldrPct < statsLdrMin)
            statsLdrMin = ldrPct;
        if (ldrPct > statsLdrMax)
            statsLdrMax = ldrPct;
        statsLdrSum += ldrPct;
        statsSampleCount++;

        // ── page rotation ──
        cycleCount++;
        if (cycleCount >= LCD_PAGE_CYCLES)
        {
            cycleCount = 0;
            currentPage = (currentPage + 1) % 2;
        }

        // ── LCD update ──
        char row0[17];
        char row1[17];

        if (currentPage == 0)
        {
            snprintf(row0, sizeof(row0), "T:%dC H:%d%%", tempI, humI);
            snprintf(row1, sizeof(row1), "L:%d  ST:%s",
                     snapS.ldrRaw, levelStr(snapA.overallLevel));
        }
        else
        {
            snprintf(row0, sizeof(row0), "T:%s  L:%s",
                     levelStr(snapA.tempLevel), levelStr(snapA.ldrLevel));
            snprintf(row1, sizeof(row1), ">> %s <<",
                     levelStr(snapA.overallLevel));
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(row0);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(row1);

        // ── serial report ──
        printf("--- Sensor Report ---\n");
        printf("Temp: %d C  | %s", tempI, levelStr(snapA.tempLevel));
        if (!snapS.dhtValid)
            printf(" [!]");
        printf("\n");
        printf("Hum:  %d %%\n", humI);
        printf("Light: %d (%d%%)  | %s  [jit=%d %s]\n",
               snapS.ldrRaw, ldrPct, levelStr(snapA.ldrLevel),
               snapS.ldrJitter, snapS.ldrConnected ? "CONNECTED" : "FLOATING!");
        printf("Alerts triggered: Temp=%lu  Light=%lu\n",
               snapA.tempAlertCount, snapA.ldrAlertCount);
        printf("Overall: %s\n", levelStr(snapA.overallLevel));

        // ── periodic statistics report ──
        if (statsSampleCount >= STATS_WINDOW_CYCLES)
        {
            int tempAvg = (int)(statsTempSum / statsSampleCount);
            int humAvg = (int)(statsHumSum / statsSampleCount);
            int ldrAvg = (int)(statsLdrSum / statsSampleCount);

            printf("--- %ds Statistics ---\n", STATS_WINDOW_CYCLES);
            printf("Temp : min=%d  max=%d  avg=%d C\n",
                   statsTempMin, statsTempMax, tempAvg);
            printf("Hum  : min=%d  max=%d  avg=%d %%\n",
                   statsHumMin, statsHumMax, humAvg);
            printf("Light: min=%d  max=%d  avg=%d %%\n",
                   statsLdrMin, statsLdrMax, ldrAvg);
            printf("-----------------\n\n");

            statsTempMin = 9999;
            statsTempMax = -9999;
            statsTempSum = 0;
            statsHumMin = 9999;
            statsHumMax = -9999;
            statsHumSum = 0;
            statsLdrMin = 101;
            statsLdrMax = -1;
            statsLdrSum = 0;
            statsSampleCount = 0;
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_DISPLAY));
    }
}

//  Task 4 — Heartbeat  (priority 1, 500 ms)

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

//  Setup — hardware init, RTOS objects, scheduler start

void appLab31Setup()
{
    srvSerialSetup();

    ddLdrSetup(PIN_LDR);
    ddDhtSetup(PIN_DHT);
    ddLcdSetup(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

    ddLedSetup(LED_GREEN, PIN_LED_GREEN);
    ddLedSetup(LED_RED, PIN_LED_RED);
    ddLedSetup(LED_YELLOW, PIN_LED_YELLOW);
    srvHeartbeatSetup(LED_HEARTBEAT, PIN_LED_HEARTBEAT);

    xSensorMutex = xSemaphoreCreateMutex();
    xAlertMutex = xSemaphoreCreateMutex();

    if (!xSensorMutex || !xAlertMutex)
    {
        printf("FATAL: mutex creation failed (heap too small?)\n");
        for (;;)
        {
        }
    }

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(taskSensorAcq, "Acquire", 256, NULL, 3, NULL);
    ok &= xTaskCreate(taskCondition, "Condition", 256, NULL, 2, NULL);
    ok &= xTaskCreate(taskDisplay, "Display", 512, NULL, 1, NULL);
    ok &= xTaskCreate(taskHeartbeat, "HB", 192, NULL, 1, NULL);

    if (ok != pdPASS)
    {
        printf("FATAL: task creation failed (heap too small?)\n");
        for (;;)
        {
        }
    }

    printf("Lab 3.1 - Dual Sensor Monitor (FreeRTOS, Variant C)\n");
    printf("Sensors: LDR (analog A0) + DHT11 (digital pin 22)\n");
    printf("Thresholds: Temp WARN>=%dC ALERT>=%dC | Light WARN>=%d%% ALERT>=%d%%\n",
           TEMP_WARN_THRESHOLD, TEMP_ALERT_THRESHOLD,
           LDR_WARN_THRESHOLD, LDR_ALERT_THRESHOLD);

    vTaskStartScheduler();
}

void appLab31Loop()
{
    // FreeRTOS scheduler is running; this is never reached.
}

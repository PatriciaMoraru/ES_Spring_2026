// Lab 3.2 - Analog Signal Acquisition with Signal Conditioning (FreeRTOS, Variant C)
//
// Sensors : LDR (analog, A0)  +  DHT11 (digital, pin 22)
// Output  : LCD 1602 (I2C)    +  Serial monitor
// Alerts  : Green / Yellow / Red LEDs with hysteresis and debounce
//
// Signal conditioning pipeline (per channel):
//   Raw sensor --> Saturation --> Median filter --> EMA --> Processed value
//
// FreeRTOS tasks:
//   1. taskAcquisition  - reads both sensors + saturation        (prio 3, 50 ms)
//   2. taskConditioning - median + EMA + threshold eval + LEDs   (prio 2, 100 ms)
//   3. taskDisplay      - LCD pages + serial report + statistics (prio 1, 500 ms)
//   4. taskHeartbeat    - heartbeat blink                        (prio 1, 500 ms)

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>

#include "app_lab_3_2.h"
#include "dd_led/dd_led.h"
#include "dd_lcd/dd_lcd.h"
#include "dd_ldr/dd_ldr.h"
#include "dd_dht/dd_dht.h"
#include "dd_button/dd_button.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"


// Pin assignments  (pins 2, 3, 5 reserved by FreeRTOS Timer 3)


#define PIN_LDR           A0
#define PIN_DHT           22
#define LCD_I2C_ADDR      0x27
#define LCD_COLS          16
#define LCD_ROWS          2
#define PIN_LED_GREEN     7
#define PIN_LED_RED       8
#define PIN_LED_YELLOW    9
#define PIN_LED_HEARTBEAT 13
#define PIN_BTN_MODE      10

#define LED_GREEN     0
#define LED_RED       1
#define LED_YELLOW    2
#define LED_HEARTBEAT 3

#define BTN_MODE  0

// Guarantees at least 1 tick even at low ms values
#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)


// Task periods (ms)


#define PERIOD_ACQUIRE   50
#define PERIOD_CONDITION 100
#define PERIOD_DISPLAY   500
#define PERIOD_HEARTBEAT 500

// DHT11 physical minimum read interval
#define DHT_READ_INTERVAL_MS 2000


// Signal conditioning parameters


#define MEDIAN_WINDOW_SIZE 5
#define EMA_ALPHA_DEFAULT  0.3f

// Filter modes (cycled by button)
typedef enum
{
    FILT_FULL,          // Median + EMA (default)
    FILT_MEDIAN_ONLY,   // Median only, no EMA
    FILT_EMA_ONLY,      // EMA only, no median
    FILT_BYPASS,        // No filtering (raw passthrough)
    FILT_MODE_COUNT
} FilterMode_t;

static const char *filterModeStr[] = {"FULL", "MED", "EMA", "OFF"};

// Runtime-configurable filter mode (single-byte write is atomic on AVR)
static volatile uint8_t filterMode = FILT_FULL;

// Saturation limits
#define SAT_LDR_RAW_MIN  0
#define SAT_LDR_RAW_MAX  1023
#define SAT_LDR_PCT_MIN  0
#define SAT_LDR_PCT_MAX  100
#define SAT_TEMP_MIN      (-10.0f)
#define SAT_TEMP_MAX      60.0f
#define SAT_HUM_MIN       0.0f
#define SAT_HUM_MAX       100.0f


// Threshold parameters


// Temperature (degrees C)
#define TEMP_WARN_THRESHOLD  28
#define TEMP_ALERT_THRESHOLD 32
#define TEMP_HYSTERESIS      2

// Light (0-100 %, higher = darker on this inverted LDR module)
#define LDR_WARN_THRESHOLD   60
#define LDR_ALERT_THRESHOLD  75
#define LDR_HYSTERESIS       5

// Consecutive readings to confirm a state change
#define DEBOUNCE_COUNT 3

// LCD page rotation: switch every N display cycles
#define LCD_PAGE_CYCLES 3

// Statistics window in display cycles (1 cycle = PERIOD_DISPLAY ms)
#define STATS_WINDOW_CYCLES 60


// Data types


typedef enum
{
    LEVEL_NORMAL,
    LEVEL_WARNING,
    LEVEL_ALERT
} AlertLevel_t;

// Raw saturated sensor readings (written by taskAcquisition)
typedef struct
{
    int     ldrRaw;
    uint8_t ldrPercent;
    int     ldrJitter;
    bool    ldrConnected;
    float   temperature;
    float   humidity;
    bool    dhtValid;
} RawData_t;

// Filtered / processed values (written by taskConditioning)
typedef struct
{
    float ldrFiltered;
    float tempFiltered;
    float humFiltered;
} ProcessedData_t;

// Alert state (written by taskConditioning)
typedef struct
{
    AlertLevel_t  tempLevel;
    AlertLevel_t  ldrLevel;
    AlertLevel_t  overallLevel;
    unsigned long tempAlertCount;
    unsigned long ldrAlertCount;
} AlertState_t;

// Median filter state for one channel
typedef struct
{
    float   buffer[MEDIAN_WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
} MedianFilter_t;

// Exponential moving average state for one channel
typedef struct
{
    float value;
    bool  initialized;
} EmaFilter_t;


// Shared data protected by mutexes


static RawData_t       rawData       = {0, 0, 0, false, 0.0f, 0.0f, false};
static ProcessedData_t processedData = {0.0f, 0.0f, 0.0f};
static AlertState_t    alertState    = {LEVEL_NORMAL, LEVEL_NORMAL, LEVEL_NORMAL, 0, 0};

static SemaphoreHandle_t xRawMutex  = NULL;   // protects rawData
static SemaphoreHandle_t xProcMutex = NULL;    // protects processedData + alertState


// Helpers


static const char *levelStr(AlertLevel_t level)
{
    switch (level)
    {
    case LEVEL_WARNING: return "WARN";
    case LEVEL_ALERT:   return "ALERT";
    default:            return "OK";
    }
}

static int saturateI(int value, int lo, int hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float saturateF(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}


// Median filter -- push sample, return current window median


static float medianUpdate(MedianFilter_t *f, float sample)
{
    f->buffer[f->index] = sample;
    f->index = (f->index + 1) % MEDIAN_WINDOW_SIZE;
    if (f->count < MEDIAN_WINDOW_SIZE)
        f->count++;

    float sorted[MEDIAN_WINDOW_SIZE];
    for (uint8_t i = 0; i < f->count; i++)
        sorted[i] = f->buffer[i];

    // Insertion sort (efficient for small N)
    for (uint8_t i = 1; i < f->count; i++)
    {
        float  key = sorted[i];
        int8_t j   = i - 1;
        while (j >= 0 && sorted[j] > key)
        {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    return sorted[f->count / 2];
}


// Exponential moving average -- update and return smoothed value


static float emaUpdate(EmaFilter_t *f, float sample, float alpha)
{
    if (!f->initialized)
    {
        f->value       = sample;
        f->initialized = true;
    }
    else
    {
        f->value = alpha * sample + (1.0f - alpha) * f->value;
    }
    return f->value;
}


// Threshold evaluation with hysteresis


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

static AlertLevel_t evalLdrLevel(float percent, AlertLevel_t current)
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


// Debounce -- only accept a new level after DEBOUNCE_COUNT consecutive agrees


static AlertLevel_t applyDebounce(AlertLevel_t  current,
                                  AlertLevel_t  candidate,
                                  AlertLevel_t *pending,
                                  uint8_t      *counter)
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


// Task 1 -- Sensor Acquisition  (priority 3, 50 ms)
//
// Reads LDR every cycle.  Reads DHT11 only every 2 s (hardware limit).
// Applies saturation to all raw values before storing.


static void taskAcquisition(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    unsigned long lastDhtReadMs = 0;
    float cachedTemp  = 0.0f;
    float cachedHum   = 0.0f;
    bool  cachedValid = false;

    for (;;)
    {
        int     rawAdc = saturateI(ddLdrRead(), SAT_LDR_RAW_MIN, SAT_LDR_RAW_MAX);
        uint8_t rawPct = (uint8_t)saturateI(ddLdrGetPercent(), SAT_LDR_PCT_MIN, SAT_LDR_PCT_MAX);

        unsigned long now = millis();
        if (now - lastDhtReadMs >= DHT_READ_INTERVAL_MS)
        {
            if (ddDhtRead())
            {
                cachedTemp  = saturateF(ddDhtGetTemperature(), SAT_TEMP_MIN, SAT_TEMP_MAX);
                cachedHum   = saturateF(ddDhtGetHumidity(),    SAT_HUM_MIN,  SAT_HUM_MAX);
                cachedValid = true;
            }
            else
            {
                cachedValid = false;
            }
            lastDhtReadMs = now;
        }

        xSemaphoreTake(xRawMutex, portMAX_DELAY);
        rawData.ldrRaw       = rawAdc;
        rawData.ldrPercent   = rawPct;
        rawData.ldrJitter    = ddLdrGetJitter();
        rawData.ldrConnected = ddLdrIsConnected();
        rawData.temperature  = cachedTemp;
        rawData.humidity     = cachedHum;
        rawData.dhtValid     = cachedValid;
        xSemaphoreGive(xRawMutex);

        // --- debounced button scanning ---
        ddButtonUpdate(BTN_MODE);

        if (ddButtonPressed(BTN_MODE))
        {
            filterMode = (filterMode + 1) % FILT_MODE_COUNT;
            printf("[BTN] Filter mode -> %s\n", filterModeStr[filterMode]);
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_ACQUIRE));
    }
}


// Task 2 -- Signal Conditioning  (priority 2, 100 ms)
//
// Pipeline: snapshot raw --> median filter --> EMA --> threshold eval + LEDs
// Flashes all LEDs briefly when the overall alert level changes.


static void taskConditioning(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    // Per-channel filter states
    MedianFilter_t medLdr  = {{0}, 0, 0};
    MedianFilter_t medTemp = {{0}, 0, 0};
    MedianFilter_t medHum  = {{0}, 0, 0};

    EmaFilter_t emaLdr  = {0.0f, false};
    EmaFilter_t emaTemp = {0.0f, false};
    EmaFilter_t emaHum  = {0.0f, false};

    // Alert debounce state
    AlertLevel_t curTemp     = LEVEL_NORMAL;
    AlertLevel_t curLdr      = LEVEL_NORMAL;
    AlertLevel_t pendingTemp = LEVEL_NORMAL;
    uint8_t      debounceTemp = 0;
    AlertLevel_t pendingLdr  = LEVEL_NORMAL;
    uint8_t      debounceLdr  = 0;

    for (;;)
    {
        // --- snapshot raw data ---
        RawData_t snap;
        xSemaphoreTake(xRawMutex, portMAX_DELAY);
        snap = rawData;
        xSemaphoreGive(xRawMutex);

        // --- configurable signal conditioning pipeline ---
        uint8_t mode  = filterMode;
        float   alpha = EMA_ALPHA_DEFAULT;

        float afterMed_Ldr, afterMed_Temp, afterMed_Hum;
        float filtLdr, filtTemp, filtHum;

        if (mode == FILT_FULL || mode == FILT_MEDIAN_ONLY)
        {
            afterMed_Ldr  = medianUpdate(&medLdr,  (float)snap.ldrPercent);
            afterMed_Temp = medianUpdate(&medTemp, snap.temperature);
            afterMed_Hum  = medianUpdate(&medHum,  snap.humidity);
        }
        else
        {
            afterMed_Ldr  = (float)snap.ldrPercent;
            afterMed_Temp = snap.temperature;
            afterMed_Hum  = snap.humidity;
        }

        if (mode == FILT_FULL || mode == FILT_EMA_ONLY)
        {
            filtLdr  = emaUpdate(&emaLdr,  afterMed_Ldr,  alpha);
            filtTemp = emaUpdate(&emaTemp, afterMed_Temp, alpha);
            filtHum  = emaUpdate(&emaHum,  afterMed_Hum,  alpha);
        }
        else
        {
            filtLdr  = afterMed_Ldr;
            filtTemp = afterMed_Temp;
            filtHum  = afterMed_Hum;
        }

        // --- threshold evaluation on filtered values ---
        AlertLevel_t candTemp = evalTempLevel(filtTemp, curTemp);
        AlertLevel_t candLdr  = evalLdrLevel(filtLdr,   curLdr);

        AlertLevel_t prevTemp = curTemp;
        AlertLevel_t prevLdr  = curLdr;

        curTemp = applyDebounce(curTemp, candTemp, &pendingTemp, &debounceTemp);
        curLdr  = applyDebounce(curLdr,  candLdr,  &pendingLdr,  &debounceLdr);

        AlertLevel_t overall = (curTemp > curLdr) ? curTemp : curLdr;

        // --- write processed + alert data ---
        xSemaphoreTake(xProcMutex, portMAX_DELAY);
        processedData.ldrFiltered  = filtLdr;
        processedData.tempFiltered = filtTemp;
        processedData.humFiltered  = filtHum;
        alertState.tempLevel       = curTemp;
        alertState.ldrLevel        = curLdr;
        if (curTemp > prevTemp) alertState.tempAlertCount++;
        if (curLdr  > prevLdr)  alertState.ldrAlertCount++;
        bool levelChanged          = (overall != alertState.overallLevel);
        alertState.overallLevel    = overall;
        xSemaphoreGive(xProcMutex);

        // --- visual feedback: flash on level change ---
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

        // Steady-state LED reflects current overall level
        ddLedOff(LED_GREEN);
        ddLedOff(LED_YELLOW);
        ddLedOff(LED_RED);

        switch (overall)
        {
        case LEVEL_NORMAL:  ddLedOn(LED_GREEN);  break;
        case LEVEL_WARNING: ddLedOn(LED_YELLOW); break;
        case LEVEL_ALERT:   ddLedOn(LED_RED);    break;
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONDITION));
    }
}


// Task 3 -- Display & Reporting  (priority 1, 500 ms)
//
// LCD: two alternating pages (filtered values / raw-vs-filtered comparison).
// Serial: full report each cycle; periodic statistics summary.


static void taskDisplay(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t cycleCount  = 0;
    uint8_t currentPage = 0;

    // Min / max / sum statistics for the current window (raw + filtered)
    int   sRawTempMin  =  9999, sRawTempMax  = -9999;
    long  sRawTempSum  = 0;
    int   sFiltTempMin =  9999, sFiltTempMax = -9999;
    long  sFiltTempSum = 0;
    int   sRawLdrMin   =  101,  sRawLdrMax   = -1;
    long  sRawLdrSum   = 0;
    int   sFiltLdrMin  =  101,  sFiltLdrMax  = -1;
    long  sFiltLdrSum  = 0;
    unsigned int statsSampleCount = 0;

    for (;;)
    {
        // --- consistent snapshot of all shared structs ---
        RawData_t       snapR;
        ProcessedData_t snapP;
        AlertState_t    snapA;

        xSemaphoreTake(xRawMutex, portMAX_DELAY);
        snapR = rawData;
        xSemaphoreGive(xRawMutex);

        xSemaphoreTake(xProcMutex, portMAX_DELAY);
        snapP = processedData;
        snapA = alertState;
        xSemaphoreGive(xProcMutex);

        // Integer representations for display (AVR printf has no %f)
        int rawTempI  = (int)snapR.temperature;
        int rawHumI   = (int)snapR.humidity;
        int rawLdrPct = (int)snapR.ldrPercent;
        int filtTempI = (int)snapP.tempFiltered;
        int filtHumI  = (int)snapP.humFiltered;
        int filtLdrI  = (int)snapP.ldrFiltered;

        // --- update statistics ---
        if (rawTempI  < sRawTempMin)  sRawTempMin  = rawTempI;
        if (rawTempI  > sRawTempMax)  sRawTempMax  = rawTempI;
        sRawTempSum  += rawTempI;
        if (filtTempI < sFiltTempMin) sFiltTempMin = filtTempI;
        if (filtTempI > sFiltTempMax) sFiltTempMax = filtTempI;
        sFiltTempSum += filtTempI;
        if (rawLdrPct < sRawLdrMin)   sRawLdrMin   = rawLdrPct;
        if (rawLdrPct > sRawLdrMax)   sRawLdrMax   = rawLdrPct;
        sRawLdrSum   += rawLdrPct;
        if (filtLdrI  < sFiltLdrMin)  sFiltLdrMin  = filtLdrI;
        if (filtLdrI  > sFiltLdrMax)  sFiltLdrMax  = filtLdrI;
        sFiltLdrSum  += filtLdrI;
        statsSampleCount++;

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
            // Page 0: filtered values + overall status
            snprintf(row0, sizeof(row0), "T:%dC H:%d%%", filtTempI, filtHumI);
            snprintf(row1, sizeof(row1), "L:%d%% ST:%s", filtLdrI, levelStr(snapA.overallLevel));
        }
        else
        {
            // Page 1: raw vs filtered comparison + per-sensor alert
            snprintf(row0, sizeof(row0), "rT:%d fT:%d %s", rawTempI, filtTempI, levelStr(snapA.tempLevel));
            snprintf(row1, sizeof(row1), "rL:%d fL:%d %s", rawLdrPct, filtLdrI, levelStr(snapA.ldrLevel));
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(row0);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(row1);

        // --- serial report ---
        printf("--- Signal Report [%s] ---\n", filterModeStr[filterMode]);
        printf("Temp : raw=%dC  filt=%dC  | %s", rawTempI, filtTempI, levelStr(snapA.tempLevel));
        if (!snapR.dhtValid)
            printf(" [!]");
        printf("\n");
        printf("Hum  : raw=%d%%  filt=%d%%\n", rawHumI, filtHumI);
        printf("Light: raw=%d (%d%%)  filt=%d%%  | %s  [jit=%d %s]\n",
               snapR.ldrRaw, rawLdrPct, filtLdrI, levelStr(snapA.ldrLevel),
               snapR.ldrJitter, snapR.ldrConnected ? "OK" : "FLOAT!");
        printf("Delta: dT=%dC  dL=%d%%\n",
               rawTempI - filtTempI, rawLdrPct - filtLdrI);
        printf("Alerts: Temp=%lu  Light=%lu\n",
               snapA.tempAlertCount, snapA.ldrAlertCount);
        printf("Overall: %s\n", levelStr(snapA.overallLevel));

        // --- Teleplot real-time visualization ---
        printf(">rawTemp:%d\n",  rawTempI);
        printf(">filtTemp:%d\n", filtTempI);
        printf(">rawHum:%d\n",   rawHumI);
        printf(">filtHum:%d\n",  filtHumI);
        printf(">rawLdr:%d\n",   rawLdrPct);
        printf(">filtLdr:%d\n",  filtLdrI);
        printf(">alertLevel:%d\n", (int)snapA.overallLevel);
        printf(">filterMode:%d\n", (int)filterMode);

        // --- periodic statistics summary ---
        if (statsSampleCount >= STATS_WINDOW_CYCLES)
        {
            int n = (int)statsSampleCount;
            printf("--- %ds Statistics ---\n",
                   (int)((unsigned long)STATS_WINDOW_CYCLES * PERIOD_DISPLAY / 1000));
            printf("Temp(raw) : min=%d  max=%d  avg=%d C\n",
                   sRawTempMin, sRawTempMax, (int)(sRawTempSum / n));
            printf("Temp(filt): min=%d  max=%d  avg=%d C\n",
                   sFiltTempMin, sFiltTempMax, (int)(sFiltTempSum / n));
            printf("LDR(raw)  : min=%d  max=%d  avg=%d %%\n",
                   sRawLdrMin, sRawLdrMax, (int)(sRawLdrSum / n));
            printf("LDR(filt) : min=%d  max=%d  avg=%d %%\n",
                   sFiltLdrMin, sFiltLdrMax, (int)(sFiltLdrSum / n));
            printf("---------------------\n\n");

            sRawTempMin  =  9999; sRawTempMax  = -9999; sRawTempSum  = 0;
            sFiltTempMin =  9999; sFiltTempMax = -9999; sFiltTempSum = 0;
            sRawLdrMin   =  101;  sRawLdrMax   = -1;    sRawLdrSum   = 0;
            sFiltLdrMin  =  101;  sFiltLdrMax  = -1;    sFiltLdrSum  = 0;
            statsSampleCount = 0;
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_DISPLAY));
    }
}


// Task 4 -- Heartbeat  (priority 1, 500 ms)


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


// Setup -- init hardware, create mutexes and tasks, start scheduler


void appLab32Setup()
{
    srvSerialSetup();

    ddLdrSetup(PIN_LDR);
    ddDhtSetup(PIN_DHT);
    ddLcdSetup(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

    ddLedSetup(LED_GREEN,     PIN_LED_GREEN);
    ddLedSetup(LED_RED,       PIN_LED_RED);
    ddLedSetup(LED_YELLOW,    PIN_LED_YELLOW);
    srvHeartbeatSetup(LED_HEARTBEAT, PIN_LED_HEARTBEAT);

    ddButtonSetup(BTN_MODE, PIN_BTN_MODE);

    xRawMutex  = xSemaphoreCreateMutex();
    xProcMutex = xSemaphoreCreateMutex();

    if (!xRawMutex || !xProcMutex)
    {
        printf("FATAL: mutex creation failed (heap too small?)\n");
        for (;;) { }
    }

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(taskAcquisition,  "Acquire",   256, NULL, 3, NULL);
    ok &= xTaskCreate(taskConditioning, "Condition",  384, NULL, 2, NULL);
    ok &= xTaskCreate(taskDisplay,      "Display",    512, NULL, 1, NULL);
    ok &= xTaskCreate(taskHeartbeat,    "HB",         192, NULL, 1, NULL);

    if (ok != pdPASS)
    {
        printf("FATAL: task creation failed (heap too small?)\n");
        for (;;) { }
    }

    printf("Lab 3.2 - Signal Acquisition & Conditioning (Variant C)\n");
    printf("Sensors: LDR (A0) + DHT11 (pin 22)\n");
    printf("Filter : Median(win=%d) + EMA(alpha=0.%d)\n",
           MEDIAN_WINDOW_SIZE, (int)(EMA_ALPHA_DEFAULT * 10));
    printf("Button : pin %d = cycle filter mode (FULL/MED/EMA/OFF)\n",
           PIN_BTN_MODE);
    printf("Thresh : Temp WARN>=%dC ALERT>=%dC | Light WARN>=%d%% ALERT>=%d%%\n",
           TEMP_WARN_THRESHOLD, TEMP_ALERT_THRESHOLD,
           LDR_WARN_THRESHOLD, LDR_ALERT_THRESHOLD);

    vTaskStartScheduler();
}

void appLab32Loop()
{
    // FreeRTOS scheduler is running; this is never reached.
}

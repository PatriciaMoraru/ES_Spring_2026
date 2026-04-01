// Lab 4.2 - Dual Actuator Control (FreeRTOS, Variant C)
//
// Actuator 1 (binary) : relay on PIN_BIN_ACT (active-LOW) — switches a red LED
// Actuator 2 (analog) : DC motor 3V via L298N (ENA=PWM, IN1, IN2)
// Input               : Serial STDIO via scanf()
// Output              : I2C LCD 1602 + Serial monitor
//
// FreeRTOS tasks:
//   1. taskCmdRead        - scanf() token parsing, routes cmds to both pipelines (event, prio 1)
//   2. taskBinActCtrl     - binary relay: on/off/toggle/timer, CMD debounce  (75 ms,  prio 3)
//   3. taskMotorCtrl      - analog motor: ramp + PWM drive                   (50 ms,  prio 3)
//   4. taskBinCond        - binary: saturation, SW-debounce, false-switch     (100 ms, prio 2)
//   5. taskMotorCond      - analog: saturation, median filter, overload alert (100 ms, prio 2)
//   6. taskDisplay        - 3-page LCD rotation + on-demand serial reports    (500 ms, prio 1)
//   7. taskHeartbeat      - heartbeat blink                                   (500 ms, prio 1)

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "app_lab_4_2.h"
#include "dd_actuator/dd_actuator.h"
#include "dd_motor/dd_motor.h"
#include "dd_lcd/dd_lcd.h"
#include "srv_serial_stdio/srv_serial_stdio.h"
#include "srv_heartbeat/srv_heartbeat.h"

#define PIN_BIN_ACT       6      // relay, active-LOW
#define PIN_MOTOR_EN      9      // L298N ENA — PWM
#define PIN_MOTOR_IN1     7      // L298N IN1
#define PIN_MOTOR_IN2     8      // L298N IN2
#define PIN_LED_HEARTBEAT 13

#define LCD_I2C_ADDR  0x27
#define LCD_COLS      16
#define LCD_ROWS      2

#define BIN_ACT_ID    0
#define MOTOR_ID      0
#define LED_HB_ID     0

#define MS_TO_TICKS(ms) \
    ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : (TickType_t)1)



#define PERIOD_BIN_CTRL  75
#define PERIOD_MOT_CTRL  50
#define PERIOD_CONDITION 100
#define PERIOD_DISPLAY   500
#define PERIOD_HEARTBEAT 500


// Binary actuator conditioning parameters


#define CMD_DEBOUNCE_MS        150
#define DEBOUNCE_COUNT         3
#define FALSE_SWITCH_THRESHOLD 5
#define FALSE_SWITCH_WINDOW_MS 5000
#define TIMER_MAX_SEC          3600
#define LCD_PAGE_CYCLES        3


// Analog motor parameters


// Hard cap: 6V supply, L298N ~2V drop → ~4V at motor.
// 3V motor rated: 75% of 4V ≈ 3V. Leaving headroom we cap at 80%.
#define MOTOR_MAX_PCT          80
#define RAMP_STEP_PCT          5     // %/cycle speed increment during ramp
#define MEDIAN_BUF_SIZE        5     // samples for median filter
#define OVERLOAD_THRESHOLD_PCT 75    // % speed considered "overload"
#define OVERLOAD_TIME_MS       3000  // ms at overload before alert fires


// Binary actuator -------------------------------------------------------

typedef enum
{
    CMD_NONE,
    CMD_ON,
    CMD_OFF,
    CMD_TOGGLE,
    CMD_TIMER,
    CMD_REPORT
} BinCmdType_t;

typedef struct
{
    BinCmdType_t type;
    uint16_t     timerSec;
} BinPendingCmd_t;

typedef struct
{
    bool     rawState;
    uint8_t  cmdCount;
    uint32_t lastChangedMs;
    char     lastCmd[8];
    bool     timerActive;
    uint16_t timerRemainSec;
} BinActData_t;

typedef struct
{
    bool    validatedState;
    bool    falseSwitchFlag;
    uint8_t falseSwitchCount;
    uint8_t debounceCounter;
} BinCondData_t;

// Analog motor ----------------------------------------------------------

typedef enum
{
    MCMD_NONE,
    MCMD_SPEED,
    MCMD_STOP,
    MCMD_FWD,
    MCMD_REV,
    MCMD_REPORT
} MotCmdType_t;

typedef struct
{
    MotCmdType_t type;
    uint8_t      speedPct;   // only used by MCMD_SPEED
} MotPendingCmd_t;

typedef struct
{
    uint8_t  targetSpeed;    // commanded speed (0-100, capped)
    bool     direction;      // true = forward
    uint8_t  cmdCount;
    uint32_t lastChangedMs;
    char     lastCmd[8];
} MotActData_t;

typedef struct
{
    uint8_t saturatedSpeed;  // after clamping
    uint8_t filteredSpeed;   // after median filter
    uint8_t rampedSpeed;     // currently applied speed (after ramp)
    bool    overloadFlag;
    uint8_t overloadCount;
} MotCondData_t;


// Shared instances + mutexes


static BinPendingCmd_t binPendingCmd  = {CMD_NONE, 0};
static BinActData_t    binActData     = {false, 0, 0, "none", false, 0};
static BinCondData_t   binCondData    = {false, false, 0, 0};

static MotPendingCmd_t motPendingCmd  = {MCMD_NONE, 0};
static MotActData_t    motActData     = {0, true, 0, 0, "none"};
static MotCondData_t   motCondData    = {0, 0, 0, false, 0};

static SemaphoreHandle_t xBinCmdMutex  = NULL;
static SemaphoreHandle_t xBinActMutex  = NULL;
static SemaphoreHandle_t xBinCondMutex = NULL;
static SemaphoreHandle_t xMotCmdMutex  = NULL;
static SemaphoreHandle_t xMotActMutex  = NULL;
static SemaphoreHandle_t xMotCondMutex = NULL;

// Atomic report-request flags (single-byte writes are atomic on AVR)
static volatile bool binReportRequested = false;
static volatile bool motReportRequested = false;


// Task 1 -- Serial command interpreter (event-driven, priority 1)
//
// Waits for whitespace-delimited tokens via scanf. Dispatches each token
// to the appropriate subsystem: relay commands -> BinPendingCmd_t,
// motor commands -> MotPendingCmd_t.  Quick-reply commands (help, status,
// mstatus) are handled directly without entering any pipeline.


static void toLower(char *s)
{
    while (*s) { if (*s >= 'A' && *s <= 'Z') *s |= 0x20; s++; }
}

static void printHelp(void)
{
    printf("--- Relay commands ---\n");
    printf("  on / off / toggle / timer N (max %d) / status / report\n", TIMER_MAX_SEC);
    printf("--- Motor commands ---\n");
    printf("  speed N (0-100%%) / stop / fwd / rev / mstatus / mreport\n");
}

static void printBinStatus(void)
{
    xSemaphoreTake(xBinActMutex, portMAX_DELAY);
    bool    isOn     = binActData.rawState;
    bool    counting = binActData.timerActive;
    uint16_t secsLeft = binActData.timerRemainSec;
    xSemaphoreGive(xBinActMutex);

    printf("[BIN] %s", isOn ? "ON" : "OFF");
    if (counting) printf("  (auto-off in %us)", secsLeft);
    printf("\n");
}

static void printMotStatus(void)
{
    xSemaphoreTake(xMotActMutex, portMAX_DELAY);
    uint8_t tgt     = motActData.targetSpeed;
    bool    forward = motActData.direction;
    xSemaphoreGive(xMotActMutex);

    xSemaphoreTake(xMotCondMutex, portMAX_DELAY);
    uint8_t curRamp   = motCondData.rampedSpeed;
    bool    isOverload = motCondData.overloadFlag;
    xSemaphoreGive(xMotCondMutex);

    printf("[MOT] target=%u%%  ramped=%u%%  dir=%s  ovl=%s\n",
           tgt, curRamp, forward ? "FWD" : "REV", isOverload ? "YES" : "NO");
}

static bool tryParseBinCmd(const char *token)
{
    BinCmdType_t action   = CMD_NONE;
    uint16_t     duration = 0;

    if      (strcmp(token, "on")     == 0) action = CMD_ON;
    else if (strcmp(token, "off")    == 0) action = CMD_OFF;
    else if (strcmp(token, "toggle") == 0) action = CMD_TOGGLE;
    else if (strcmp(token, "report") == 0) action = CMD_REPORT;
    else if (strcmp(token, "timer")  == 0)
    {
        unsigned int secs = 0;
        if (scanf("%u", &secs) == 1 && secs >= 1 && secs <= TIMER_MAX_SEC)
        {
            duration = (uint16_t)secs;
            action   = CMD_TIMER;
        }
        else
        {
            printf("[ERR] Usage: timer <seconds>  (1-%d)\n", TIMER_MAX_SEC);
            return true;
        }
    }

    if (action == CMD_NONE)
        return false;

    xSemaphoreTake(xBinCmdMutex, portMAX_DELAY);
    binPendingCmd.type     = action;
    binPendingCmd.timerSec = duration;
    xSemaphoreGive(xBinCmdMutex);
    return true;
}

static bool tryParseMotCmd(const char *token)
{
    MotCmdType_t action    = MCMD_NONE;
    uint8_t      pctValue  = 0;

    if      (strcmp(token, "stop")    == 0) action = MCMD_STOP;
    else if (strcmp(token, "fwd")     == 0) action = MCMD_FWD;
    else if (strcmp(token, "rev")     == 0) action = MCMD_REV;
    else if (strcmp(token, "mreport") == 0) action = MCMD_REPORT;
    else if (strcmp(token, "speed")   == 0)
    {
        unsigned int val = 0;
        if (scanf("%u", &val) == 1 && val <= 100)
        {
            pctValue = (uint8_t)val;
            action   = MCMD_SPEED;
        }
        else
        {
            printf("[ERR] Usage: speed <0-100>\n");
            return true;
        }
    }

    if (action == MCMD_NONE)
        return false;

    xSemaphoreTake(xMotCmdMutex, portMAX_DELAY);
    motPendingCmd.type     = action;
    motPendingCmd.speedPct = pctValue;
    xSemaphoreGive(xMotCmdMutex);
    return true;
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

        if (strcmp(token, "help") == 0)    { printHelp();      continue; }
        if (strcmp(token, "status") == 0)  { printBinStatus(); continue; }
        if (strcmp(token, "mstatus") == 0) { printMotStatus(); continue; }

        if (tryParseBinCmd(token)) continue;
        if (tryParseMotCmd(token)) continue;

        printf("[ERR] Unknown command '%s'. Type 'help'.\n", token);
    }
}


// Task 2 -- Relay driver (periodic 75 ms, priority 3)
//
// Picks up one pending relay command per cycle. Applies rate-limit
// debouncing (CMD_DEBOUNCE_MS) so rapid-fire commands don't chatter
// the relay. Also manages an auto-off countdown timer.


static void updateBinSharedData(bool state, const char *tag, uint32_t ts,
                                bool tmrOn, uint16_t tmrSec)
{
    xSemaphoreTake(xBinActMutex, portMAX_DELAY);
    binActData.rawState       = state;
    binActData.cmdCount++;
    binActData.lastChangedMs  = ts;
    binActData.timerActive    = tmrOn;
    binActData.timerRemainSec = tmrSec;
    strncpy(binActData.lastCmd, tag, sizeof(binActData.lastCmd) - 1);
    binActData.lastCmd[sizeof(binActData.lastCmd) - 1] = '\0';
    xSemaphoreGive(xBinActMutex);
}

static void driveRelay(bool desired)
{
    if (desired) ddActuatorOn(BIN_ACT_ID);
    else         ddActuatorOff(BIN_ACT_ID);
}

static void taskBinActCtrl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint32_t prevCmdTs     = 0;
    uint32_t countdownEnd  = 0;
    bool     countdownOn   = false;

    for (;;)
    {
        // grab & clear the pending command
        xSemaphoreTake(xBinCmdMutex, portMAX_DELAY);
        BinPendingCmd_t incoming = binPendingCmd;
        binPendingCmd.type = CMD_NONE;
        xSemaphoreGive(xBinCmdMutex);

        // report request bypasses the actuator logic
        if (incoming.type == CMD_REPORT)
        {
            binReportRequested = true;
        }
        else if (incoming.type != CMD_NONE)
        {
            uint32_t ts = (uint32_t)millis();

            if ((ts - prevCmdTs) < CMD_DEBOUNCE_MS)
            {
                printf("[BIN] cmd ignored (debounce)\n");
            }
            else
            {
                prevCmdTs = ts;
                bool current  = ddActuatorGetState(BIN_ACT_ID);
                bool desired  = current;
                const char *tag = "?";

                switch (incoming.type)
                {
                    case CMD_ON:
                        desired = true;  tag = "on";     countdownOn = false;
                        break;
                    case CMD_OFF:
                        desired = false; tag = "off";    countdownOn = false;
                        break;
                    case CMD_TOGGLE:
                        desired = !current; tag = "toggle"; countdownOn = false;
                        break;
                    case CMD_TIMER:
                        desired      = true;
                        tag          = "timer";
                        countdownEnd = ts + (uint32_t)incoming.timerSec * 1000UL;
                        countdownOn  = true;
                        printf("[BIN] timer started — auto-off in %us\n", incoming.timerSec);
                        break;
                    default:
                        break;
                }

                driveRelay(desired);

                if (incoming.type != CMD_TIMER)
                    printf("[BIN] %s -> %s\n", tag, desired ? "ON" : "OFF");

                uint16_t secsLeft = 0;
                if (countdownOn)
                    secsLeft = (uint16_t)((countdownEnd - ts) / 1000UL);

                updateBinSharedData(desired, tag, ts, countdownOn, secsLeft);
            }
        }

        // auto-off countdown
        if (countdownOn)
        {
            uint32_t ts = (uint32_t)millis();

            if (ts >= countdownEnd)
            {
                countdownOn = false;
                driveRelay(false);
                updateBinSharedData(false, "t-exp", ts, false, 0);
                printf("[BIN] timer expired -> OFF\n");
            }
            else
            {
                uint16_t secsLeft = (uint16_t)((countdownEnd - ts) / 1000UL);
                xSemaphoreTake(xBinActMutex, portMAX_DELAY);
                binActData.timerRemainSec = secsLeft;
                xSemaphoreGive(xBinActMutex);
            }
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_BIN_CTRL));
    }
}


// Task 3 -- PWM motor driver (periodic 50 ms, priority 3)
//
// Consumes one queued motor command per cycle, resolving it into a
// target speed percentage and a rotation direction. The resolved
// values are published to motActData; the conditioning task reads
// them, applies filtering / ramping, and drives the actual hardware.


static void commitMotorState(uint8_t pct, bool fwd, const char *tag, uint32_t ts)
{
    xSemaphoreTake(xMotActMutex, portMAX_DELAY);
    motActData.targetSpeed   = pct;
    motActData.direction     = fwd;
    motActData.cmdCount++;
    motActData.lastChangedMs = ts;
    strncpy(motActData.lastCmd, tag, sizeof(motActData.lastCmd) - 1);
    motActData.lastCmd[sizeof(motActData.lastCmd) - 1] = '\0';
    xSemaphoreGive(xMotActMutex);
}

static void taskMotorCtrl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        // fetch & clear queued command
        xSemaphoreTake(xMotCmdMutex, portMAX_DELAY);
        MotPendingCmd_t incoming = motPendingCmd;
        motPendingCmd.type = MCMD_NONE;
        xSemaphoreGive(xMotCmdMutex);

        if (incoming.type == MCMD_REPORT)
        {
            motReportRequested = true;
        }
        else if (incoming.type != MCMD_NONE)
        {
            // read current motor state to preserve unchanged fields
            xSemaphoreTake(xMotActMutex, portMAX_DELAY);
            uint8_t prevPct = motActData.targetSpeed;
            bool    prevFwd = motActData.direction;
            xSemaphoreGive(xMotActMutex);

            uint32_t    ts      = (uint32_t)millis();
            uint8_t     setPct  = prevPct;
            bool        setFwd  = prevFwd;
            const char *tag     = "?";

            switch (incoming.type)
            {
                case MCMD_SPEED:
                    setPct = incoming.speedPct;
                    tag    = "speed";
                    printf("[MOT] speed -> %u%%\n", setPct);
                    break;

                case MCMD_STOP:
                    setPct = 0;
                    tag    = "stop";
                    printf("[MOT] stop\n");
                    break;

                case MCMD_FWD:
                    setFwd = true;
                    tag    = "fwd";
                    printf("[MOT] direction -> FWD\n");
                    break;

                case MCMD_REV:
                    setFwd = false;
                    tag    = "rev";
                    printf("[MOT] direction -> REV\n");
                    break;

                default:
                    break;
            }

            commitMotorState(setPct, setFwd, tag, ts);
        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_MOT_CTRL));
    }
}


// Task 4 -- Relay signal conditioning (100 ms, priority 2)
//
// Processes the raw relay state through three conditioning stages:
//   Stage A : boolean saturation — force value into {0,1}
//   Stage B : multi-cycle debounce — new value must hold for
//             DEBOUNCE_COUNT consecutive samples before it's accepted
//   Stage C : false-switch detector — ring buffer of transition
//             timestamps; if more than FALSE_SWITCH_THRESHOLD occur
//             inside FALSE_SWITCH_WINDOW_MS, an alert is raised


static uint8_t countRecentEdges(const uint32_t *ring, uint8_t capacity,
                                uint8_t head, uint8_t stored, uint32_t now)
{
    uint8_t hits = 0;
    for (uint8_t k = 0; k < stored; k++)
    {
        uint8_t pos = (head + capacity - stored + k) % capacity;
        if ((now - ring[pos]) <= FALSE_SWITCH_WINDOW_MS)
            hits++;
    }
    return hits;
}

static void taskBinCond(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    // Stage B state
    bool    candidateVal = false;
    uint8_t stableCycles = 0;
    bool    confirmedVal = false;

    // Stage C state — ring buffer of edge timestamps
    const uint8_t RING_CAP = FALSE_SWITCH_THRESHOLD + 2;
    static uint32_t edgeRing[FALSE_SWITCH_THRESHOLD + 2];
    uint8_t ringHead   = 0;
    uint8_t ringStored = 0;

    bool    fsAlert      = false;
    uint8_t fsTotalCount = 0;
    bool    prevRaw      = false;

    for (;;)
    {
        // snapshot the actuator state
        BinActData_t snapshot;
        xSemaphoreTake(xBinActMutex, portMAX_DELAY);
        snapshot = binActData;
        xSemaphoreGive(xBinActMutex);

        // Stage A: boolean saturation
        bool clamped = (snapshot.rawState != false);

        // Stage B: multi-cycle debounce
        if (clamped == candidateVal)
        {
            if (stableCycles < DEBOUNCE_COUNT) stableCycles++;
        }
        else
        {
            candidateVal = clamped;
            stableCycles = 1;
        }

        bool accepted = confirmedVal;
        if (stableCycles >= DEBOUNCE_COUNT)
            accepted = candidateVal;

        // Stage C: false-switch detection on raw edges
        if (clamped != prevRaw)
        {
            uint32_t ts = (uint32_t)millis();
            edgeRing[ringHead] = ts;
            ringHead = (ringHead + 1) % RING_CAP;
            if (ringStored < RING_CAP) ringStored++;

            uint8_t recentEdges = countRecentEdges(edgeRing, RING_CAP,
                                                   ringHead, ringStored, ts);

            if (recentEdges > FALSE_SWITCH_THRESHOLD)
            {
                if (!fsAlert) fsTotalCount++;
                fsAlert = true;
                printf("[BIN-COND] FALSE-SWITCH! %d transitions in %dms\n",
                       recentEdges, FALSE_SWITCH_WINDOW_MS);
            }
            else
            {
                fsAlert = false;
            }

            prevRaw = clamped;
        }

        confirmedVal = accepted;

        xSemaphoreTake(xBinCondMutex, portMAX_DELAY);
        binCondData.validatedState   = accepted;
        binCondData.falseSwitchFlag  = fsAlert;
        binCondData.falseSwitchCount = fsTotalCount;
        binCondData.debounceCounter  = stableCycles;
        xSemaphoreGive(xBinCondMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONDITION));
    }
}


// Task 5 -- Motor signal conditioning (100 ms, priority 2)
//
// Processes the commanded speed through three conditioning stages:
//   Stage A : saturation — clamp to [0, MOTOR_MAX_PCT]
//   Stage B : impulse rejection — 5-sample sliding median
//   Stage C : smooth ramp — step toward filtered value by RAMP_STEP_PCT/cycle
// After conditioning, the final PWM value is applied to the L298N driver.
// An overload watchdog fires if the output stays at or above
// OVERLOAD_THRESHOLD_PCT for longer than OVERLOAD_TIME_MS.


static void sortBuf(uint8_t *arr, uint8_t len)
{
    for (uint8_t i = 1; i < len; i++)
    {
        uint8_t val = arr[i];
        int8_t  p   = (int8_t)i - 1;
        while (p >= 0 && arr[p] > val) { arr[p + 1] = arr[p]; p--; }
        arr[p + 1] = val;
    }
}

static uint8_t pickMedian(uint8_t *samples)
{
    uint8_t sorted[MEDIAN_BUF_SIZE];
    memcpy(sorted, samples, MEDIAN_BUF_SIZE);
    sortBuf(sorted, MEDIAN_BUF_SIZE);
    return sorted[MEDIAN_BUF_SIZE / 2];
}

static uint8_t rampToward(uint8_t current, uint8_t goal, uint8_t maxStep)
{
    if (current < goal)
    {
        uint8_t gap = goal - current;
        return current + (gap > maxStep ? maxStep : gap);
    }
    if (current > goal)
    {
        uint8_t gap = current - goal;
        return current - (gap > maxStep ? maxStep : gap);
    }
    return current;
}

static void taskMotorCond(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    // Stage B — sliding window
    uint8_t sampleRing[MEDIAN_BUF_SIZE] = {0};
    uint8_t ringPos = 0;

    // Stage C — ramp output
    uint8_t outputPct = 0;

    // Overload watchdog
    bool     ovlActive    = false;
    bool     ovlTriggered = false;
    uint8_t  ovlTotal     = 0;
    uint32_t ovlSinceMs   = 0;

    for (;;)
    {
        MotActData_t snapshot;
        xSemaphoreTake(xMotActMutex, portMAX_DELAY);
        snapshot = motActData;
        xSemaphoreGive(xMotActMutex);

        // Stage A: clamp to safe range
        uint8_t capped = snapshot.targetSpeed;
        if (capped > MOTOR_MAX_PCT) capped = MOTOR_MAX_PCT;

        // Stage B: push into ring, extract median
        sampleRing[ringPos] = capped;
        ringPos = (ringPos + 1) % MEDIAN_BUF_SIZE;
        uint8_t smoothed = pickMedian(sampleRing);

        // Stage C: gradual ramp
        outputPct = rampToward(outputPct, smoothed, RAMP_STEP_PCT);

        // drive motor hardware
        if (outputPct == 0)
            ddMotorStop(MOTOR_ID);
        else
            ddMotorSetSpeed(MOTOR_ID, outputPct, snapshot.direction);

        // overload watchdog
        uint32_t ts = (uint32_t)millis();
        if (outputPct >= OVERLOAD_THRESHOLD_PCT)
        {
            if (!ovlActive) { ovlActive = true; ovlSinceMs = ts; }
            else if ((ts - ovlSinceMs) > OVERLOAD_TIME_MS && !ovlTriggered)
            {
                ovlTotal++;
                ovlTriggered = true;
                printf("[MOT-COND] OVERLOAD ALERT! speed=%u%% for >%dms (total=%u)\n",
                       outputPct, OVERLOAD_TIME_MS, ovlTotal);
            }
        }
        else
        {
            ovlActive    = false;
            ovlTriggered = false;
        }

        xSemaphoreTake(xMotCondMutex, portMAX_DELAY);
        motCondData.saturatedSpeed = capped;
        motCondData.filteredSpeed  = smoothed;
        motCondData.rampedSpeed    = outputPct;
        motCondData.overloadFlag   = ovlTriggered;
        motCondData.overloadCount  = ovlTotal;
        xSemaphoreGive(xMotCondMutex);

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_CONDITION));
    }
}


// Task 6 -- Display & Reporting  (priority 1, 500 ms)
//
// LCD rotates through 3 pages every LCD_PAGE_CYCLES cycles (1.5 s/page):
//   Page 0: binary actuator raw + validated state / cmd count + false-switch count
//   Page 1: motor speed (ramped) + direction / overload count + filter speed
//   Page 2: binary timer / overload or false-switch alert
// Serial reports printed on demand via "report" and "mreport" commands.


static void taskDisplay(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    uint8_t cycleCount  = 0;
    uint8_t currentPage = 0;

    for (;;)
    {
        BinActData_t  snapBA;
        BinCondData_t snapBC;
        MotActData_t  snapMA;
        MotCondData_t snapMC;

        xSemaphoreTake(xBinActMutex,  portMAX_DELAY); snapBA = binActData;  xSemaphoreGive(xBinActMutex);
        xSemaphoreTake(xBinCondMutex, portMAX_DELAY); snapBC = binCondData; xSemaphoreGive(xBinCondMutex);
        xSemaphoreTake(xMotActMutex,  portMAX_DELAY); snapMA = motActData;  xSemaphoreGive(xMotActMutex);
        xSemaphoreTake(xMotCondMutex, portMAX_DELAY); snapMC = motCondData; xSemaphoreGive(xMotCondMutex);

        uint32_t binAgeMs  = (uint32_t)millis() - snapBA.lastChangedMs;
        uint16_t binAgeSec = (uint16_t)(binAgeMs / 1000UL);

        // page rotation
        cycleCount++;
        if (cycleCount >= LCD_PAGE_CYCLES)
        {
            cycleCount  = 0;
            currentPage = (uint8_t)((currentPage + 1) % 3);
        }

        char row0[17];
        char row1[17];

        if (currentPage == 0)
        {
            // Page 0: binary actuator state
            snprintf(row0, sizeof(row0), "BIN:%-3s V:%-3s",
                     snapBA.rawState       ? "ON" : "OFF",
                     snapBC.validatedState ? "ON" : "OFF");
            snprintf(row1, sizeof(row1), "Cmds:%-3u SW:%-2u",
                     snapBA.cmdCount, snapBC.falseSwitchCount);
        }
        else if (currentPage == 1)
        {
            // Page 1: motor state
            snprintf(row0, sizeof(row0), "MTR:%-3u%% %-3s",
                     snapMC.rampedSpeed,
                     snapMA.direction ? "FWD" : "REV");
            snprintf(row1, sizeof(row1), "Tgt:%-3u Flt:%-3u",
                     snapMC.saturatedSpeed, snapMC.filteredSpeed);
        }
        else
        {
            // Page 2: alerts / timers
            if (snapBA.timerActive)
                snprintf(row0, sizeof(row0), "BIN off in:%-4us", snapBA.timerRemainSec);
            else
                snprintf(row0, sizeof(row0), "BIN age:%-4us", binAgeSec);

            uint8_t alerts = snapBC.falseSwitchCount + snapMC.overloadCount;
            snprintf(row1, sizeof(row1), "FS:%-2u OVL:%-3u",
                     snapBC.falseSwitchCount, snapMC.overloadCount);
            (void)alerts;
        }

        ddLcdClear();
        ddLcdSetCursor(0, 0);
        ddLcdPrint(row0);
        ddLcdSetCursor(0, 1);
        ddLcdPrint(row1);

        // --- on-demand binary report ---
        if (binReportRequested)
        {
            binReportRequested = false;
            printf("--- Binary Actuator Report ---\n");
            printf("Raw state   : %s\n", snapBA.rawState       ? "ON" : "OFF");
            printf("Validated   : %s\n", snapBC.validatedState ? "ON" : "OFF");
            printf("Last cmd    : %s\n", snapBA.lastCmd);
            printf("Cmd count   : %u\n", snapBA.cmdCount);
            printf("State age   : %us\n", binAgeSec);
            if (snapBA.timerActive)
                printf("Timer       : auto-off in %us\n", snapBA.timerRemainSec);
            else
                printf("Timer       : inactive\n");
            printf("Debounce cnt: %u/%d\n", snapBC.debounceCounter, DEBOUNCE_COUNT);
            if (snapBC.falseSwitchFlag)
                printf("ALERT       : false-switch! (total=%u)\n", snapBC.falseSwitchCount);
            else
                printf("Alert       : none\n");

        }

        // --- on-demand motor report ---
        if (motReportRequested)
        {
            motReportRequested = false;
            printf("--- Motor Report ---\n");
            printf("Target speed : %u%%\n",  snapMC.saturatedSpeed);
            printf("Filtered spd : %u%%\n",  snapMC.filteredSpeed);
            printf("Ramped speed : %u%%\n",  snapMC.rampedSpeed);
            printf("Direction    : %s\n",    snapMA.direction ? "FWD" : "REV");
            printf("Last cmd     : %s\n",    snapMA.lastCmd);
            printf("Cmd count    : %u\n",    snapMA.cmdCount);
            printf("Max cap      : %d%%\n",  MOTOR_MAX_PCT);
            if (snapMC.overloadFlag)
                printf("OVERLOAD     : YES (total=%u)\n", snapMC.overloadCount);
            else
                printf("Overload     : NO\n");

        }

        vTaskDelayUntil(&xLastWake, MS_TO_TICKS(PERIOD_DISPLAY));
    }
}


// Task 7 -- Heartbeat  (priority 1, 500 ms)


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


// Configure serial, relay, motor driver, heartbeat LED, and LCD hardware.
static void initPeripherals(void)
{
    srvSerialSetup();
    ddActuatorSetup(BIN_ACT_ID, PIN_BIN_ACT, true);
    ddMotorSetup(MOTOR_ID, PIN_MOTOR_EN, PIN_MOTOR_IN1, PIN_MOTOR_IN2);
    srvHeartbeatSetup(LED_HB_ID, PIN_LED_HEARTBEAT);
    ddLcdSetup(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
}

// Allocate the six FreeRTOS mutexes that guard shared data between tasks.
// Halts execution if any allocation fails (insufficient heap).
static void initMutexes(void)
{
    xBinCmdMutex  = xSemaphoreCreateMutex();
    xBinActMutex  = xSemaphoreCreateMutex();
    xBinCondMutex = xSemaphoreCreateMutex();
    xMotCmdMutex  = xSemaphoreCreateMutex();
    xMotActMutex  = xSemaphoreCreateMutex();
    xMotCondMutex = xSemaphoreCreateMutex();

    bool allValid = xBinCmdMutex && xBinActMutex && xBinCondMutex
                 && xMotCmdMutex && xMotActMutex && xMotCondMutex;

    if (!allValid)
    {
        printf("ERR: one or more mutexes could not be allocated\n");
        for (;;) { }
    }
}

// Create all seven FreeRTOS tasks (serial input, relay control,
// motor control, two conditioning tasks, display, heartbeat).
// Halts if any task cannot be created.
static void spawnTasks(void)
{
    BaseType_t result = pdPASS;
    result &= xTaskCreate(taskSerialInput, "SerIn",    256, NULL, 1, NULL);
    result &= xTaskCreate(taskBinActCtrl,  "RelayDrv", 320, NULL, 3, NULL);
    result &= xTaskCreate(taskMotorCtrl,   "MotorDrv", 256, NULL, 3, NULL);
    result &= xTaskCreate(taskBinCond,     "RelayCnd", 320, NULL, 2, NULL);
    result &= xTaskCreate(taskMotorCond,   "MotorCnd", 320, NULL, 2, NULL);
    result &= xTaskCreate(taskDisplay,     "LcdRpt",   512, NULL, 1, NULL);
    result &= xTaskCreate(taskHeartbeat,   "Pulse",    192, NULL, 1, NULL);

    if (result != pdPASS)
    {
        printf("ERR: could not spawn one or more tasks\n");
        for (;;) { }
    }
}

// Print startup summary to serial: pin assignments, PWM cap, and task periods.
static void printBanner(void)
{
    printf("=== Lab 4.2  Dual Actuator Control (Variant C) ===\n");
    printf("Relay  : pin %d (active-LOW)\n", PIN_BIN_ACT);
    printf("Motor  : L298N  EN=pin%d  IN1=pin%d  IN2=pin%d  cap=%d%%\n",
           PIN_MOTOR_EN, PIN_MOTOR_IN1, PIN_MOTOR_IN2, MOTOR_MAX_PCT);
    printf("Timing : relay %dms | motor %dms | cond %dms | lcd %dms\n",
           PERIOD_BIN_CTRL, PERIOD_MOT_CTRL, PERIOD_CONDITION, PERIOD_DISPLAY);
    printf("Send 'help' to see the command list.\n");
}

// Main entry point — called from setup(). Initialises everything
// and starts the FreeRTOS scheduler (does not return).
void appLab42Setup()
{
    initPeripherals();
    initMutexes();
    spawnTasks();
    printBanner();
    vTaskStartScheduler();
}

void appLab42Loop()
{
    // FreeRTOS scheduler is running; never reached.
}

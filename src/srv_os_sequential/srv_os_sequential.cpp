// Sequential (non-preemptive) OS service
// Counter-based scheduler with 1ms tick from Timer1.
// ISR increments a tick counter; srvOsSchedulerLoop() runs tasks from main context.
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "srv_os_sequential.h"

static SrvOsTask tasks[SRV_OS_MAX_TASKS];
static int taskCount = 0;

// IMPORTANT: counter, not bool -> prevents losing ticks if tasks take longer than 1ms
static volatile uint16_t tickCount = 0;

// Timer1 CTC interrupt: increments tick count every 1ms
ISR(TIMER1_COMPA_vect)
{
    tickCount++;
}

// Configure Timer1 for 1ms CTC interrupt (16MHz clock, prescaler 8)
static void timer1Init(void)
{
    cli();

    TCCR1A = 0;
    TCCR1B = 0;

    // CTC mode (WGM12=1), prescaler 8 (CS11=1)
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // (16MHz / 8) / 1000Hz - 1 = 1999
    OCR1A = 1999;

    // Enable compare match A interrupt
    TIMSK1 = (1 << OCIE1A);

    sei();
}

void srvOsSetup(void)
{
    taskCount = 0;

    // Clear pending ticks safely
    cli();
    tickCount = 0;
    sei();

    timer1Init();
}

void srvOsRegisterTask(SrvOsTaskFunc func, int rec, int offset)
{
    if (taskCount >= SRV_OS_MAX_TASKS || func == NULL)
    {
        return;
    }

    if (rec <= 0 || offset < 0)
    {
        return;
    }

    SrvOsTask *t = &tasks[taskCount];
    t->task_func = func;
    t->rec = rec;
    t->offset = offset;
    t->rec_cnt = offset; // start after offset ticks

    taskCount++;
}

// Call from loop(). On each pending 1ms tick, decrements counters and runs ready tasks.
void srvOsSchedulerLoop(void)
{
    // Take a snapshot of how many ticks occurred since last loop, atomically.
    uint16_t pendingTicks;

    cli();
    pendingTicks = tickCount;
    tickCount = 0;
    sei();

    if (pendingTicks == 0)
    {
        return;
    }

    while (pendingTicks--)
    {
        for (int i = 0; i < taskCount; i++)
        {
            if (--tasks[i].rec_cnt <= 0)
            {
                tasks[i].rec_cnt = tasks[i].rec;
                tasks[i].task_func();

#if SRV_OS_ONE_TASK_PER_TICK
                // Enforce "one active task per tick"
                break;
#endif
            }
        }
    }
}
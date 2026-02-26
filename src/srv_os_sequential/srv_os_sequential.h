// Sequential (non-preemptive) OS service
// Cooperative counter-based scheduler driven by a 1ms Timer1 tick.
#ifndef SRV_OS_SEQUENTIAL_H
#define SRV_OS_SEQUENTIAL_H

#include <stdint.h>

#define SRV_OS_MAX_TASKS 8

// If set to 1 -> run at most ONE ready task per tick (matches "one task active per tick").
// If set to 0 -> run ALL ready tasks per tick.
#ifndef SRV_OS_ONE_TASK_PER_TICK
#define SRV_OS_ONE_TASK_PER_TICK 1
#endif

typedef void (*SrvOsTaskFunc)(void);

typedef struct
{
    SrvOsTaskFunc task_func; // task entry function
    int rec;                 // recurrence period in ticks (ms)
    int offset;              // initial delay in ticks (ms)
    int rec_cnt;             // internal counter
} SrvOsTask;

void srvOsSetup(void);
void srvOsRegisterTask(SrvOsTaskFunc func, int rec, int offset);

// Call from Arduino loop(). Processes pending ticks and dispatches tasks from main context.
void srvOsSchedulerLoop(void);

#endif // SRV_OS_SEQUENTIAL_H
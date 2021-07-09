//
// Created by wirano on 2021/4/27.
//

#ifndef ANO_SCHEDULER_H
#define ANO_SCHEDULER_H

#include "stm32f4xx.h"

#define TICK_PER_SECOND	1000

typedef struct
{
    void(*task_func)(void);
    uint16_t rate_hz;
    uint16_t interval_ticks;
    uint32_t last_run;
}sched_task_t;

void Scheduler_Setup(void);
void Scheduler_Run(void);

extern uint16_t distance;

#endif //ANO_SCHEDULER_H

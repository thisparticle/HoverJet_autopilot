#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"


typedef struct
{
void(*task_func)(void);  ///任务函数指针
uint16_t rate_hz;
uint16_t interval_ticks;  ///任务执行周期
uint32_t last_run;  ///上次执行任务的时间
}sched_task_t;

void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif


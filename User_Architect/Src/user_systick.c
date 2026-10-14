/* 包含头文件 */
#include "../user_systick.h"

/* 私有变量 */
static SysTick_Task *systick_tasks[MAX_SYSTICK_TASK] = {0};
static uint8_t task_count = 0;

/* 函数实现 */

void SysTick_InitTask(SysTick_Task *Task, void *arg, const uint32_t delay, const uint32_t period, const TaskMode mode, const SysTick_Callback callback) {
    Task->arg = arg;
    Task->delay = delay;
    Task->period = period;
    Task->counter = 0;
    Task->mode = mode;
    Task->state = Task_STOPPED;
    Task->callback = callback;

    systick_tasks[task_count] = Task;
    task_count++;
}

void SysTick_StartTask(SysTick_Task *Task) {
    Task->state = Task_RUNNING;
}

void SysTick_StopTask(SysTick_Task *Task) {
    Task->state = Task_STOPPED;
}

void SysTick_ResetTask(SysTick_Task *Task) {
    Task->counter = 0;
    Task->state = Task_RUNNING;
}

// 该函数需要在 SysTick_Handler 中调用
void SysTick_Handle(void) {
    for (uint8_t i = 0; i < task_count; i++) {
        SysTick_Task *task = systick_tasks[i];
        
        if (task->state == Task_STOPPED) {
            continue;
        }

        task->counter++;

        if (task->counter >= task->delay) {
            task->callback((void *)task->arg);

            switch (task->mode) {
                case Task_ONCE:
                    task->state = Task_STOPPED;
                    break;
                case Task_REPEAT:
                    task->counter = 0;
                    task->delay = task->period;
                    break;
            }
        }
    }
}
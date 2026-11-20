#include "chassis_link.h"
#include "cmsis_os.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

static unsigned creates, updates, delays;
static osThreadDef_t task;
static jmp_buf stop_task;

osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument)
{
    assert(argument == NULL);
    assert(definition->pthread && definition->tpriority == osPriorityNormal);
    assert(definition->instances == 0 && definition->stacksize == 512);
    task = *definition;
    ++creates;
    return creates == 1 ? NULL : &task;
}

void ChassisLinkUpdate(void)
{
    assert(updates == delays); /* Each pass forwards before yielding. */
    ++updates;
}

void osDelay(uint32_t milliseconds)
{
    assert(milliseconds == 1 && updates == delays + 1);
    if (++delays == 3) longjmp(stop_task, 1);
}

int main(void)
{
    ChassisLinkStartTask(); /* A failed allocation may be retried. */
    assert(creates == 1 && updates == 0);
    ChassisLinkStartTask();
    ChassisLinkStartTask();
    assert(creates == 2 && updates == 0); /* Never create two running tasks. */
    if (!setjmp(stop_task)) task.pthread(NULL);
    assert(updates == 3 && delays == 3);
    puts("PASS chassis_link task: single instance, allocation retry, normal priority, original stack, 1 ms forwarding/yield");
}

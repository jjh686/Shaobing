#include "chassis_link.h"
#include "chassis_link_config.h"
#include "bsp_log.h"
#include "cmsis_os.h"

static osThreadId chassis_link_task_handle;

static __attribute__((noreturn)) void ChassisLinkTask(void const *argument)
{
    (void)argument;
    LOGINFO("[freeRTOS] NAV communication Task Start");
    for (;;)
    {
        ChassisLinkUpdate();
        osDelay(CHASSIS_LINK_TASK_PERIOD_MS);
    }
}

void ChassisLinkStartTask(void)
{
    if (chassis_link_task_handle)
        return;
    osThreadDef(navcommtask, ChassisLinkTask, osPriorityNormal, 0, CHASSIS_LINK_TASK_STACK_SIZE);
    chassis_link_task_handle = osThreadCreate(osThread(navcommtask), NULL);
}

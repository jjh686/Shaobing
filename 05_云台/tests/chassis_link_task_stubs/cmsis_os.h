#ifndef TEST_CHASSIS_LINK_CMSIS_OS_H
#define TEST_CHASSIS_LINK_CMSIS_OS_H
#include <stddef.h>
#include <stdint.h>

typedef void *osThreadId;
typedef enum { osPriorityNormal = 0 } osPriority;
typedef struct {
    void (*pthread)(void const *);
    osPriority tpriority;
    uint32_t instances, stacksize;
} osThreadDef_t;
#define osThreadDef(name, thread, priority, count, stack) \
    static const osThreadDef_t os_thread_def_##name = {thread, priority, count, stack}
#define osThread(name) (&os_thread_def_##name)
osThreadId osThreadCreate(const osThreadDef_t *, void *);
void osDelay(uint32_t);

#endif

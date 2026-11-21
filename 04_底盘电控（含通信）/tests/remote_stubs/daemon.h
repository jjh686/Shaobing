#ifndef REMOTE_TEST_DAEMON_H
#define REMOTE_TEST_DAEMON_H
#include <stddef.h>
typedef struct {int unused;} DaemonInstance;
typedef struct {unsigned reload_count;void (*callback)(void *);void *owner_id;} Daemon_Init_Config_s;
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *);
void DaemonReload(DaemonInstance *);
int DaemonIsOnline(DaemonInstance *);
#endif

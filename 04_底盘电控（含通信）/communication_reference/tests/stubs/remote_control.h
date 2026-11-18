#ifndef STUB_REMOTE_H
#define STUB_REMOTE_H
#include <stdint.h>
#define FS_RC_SW_MID 1
typedef struct {int16_t channel[10];} FS_RC_ctrl_t;
extern int huart3;
FS_RC_ctrl_t *FS_RemoteControlInit(int *);
uint8_t FS_RemoteControlIsOnline(void);
#endif

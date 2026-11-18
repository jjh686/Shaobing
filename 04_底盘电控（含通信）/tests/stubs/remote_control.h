#ifndef STUB_REMOTE_H
#define STUB_REMOTE_H
#include <stdint.h>
#define FS_RC_SW_MID 1
typedef struct {
 int16_t channel[10];uint32_t fault_generation,received_ms;
 struct {int16_t rocker_r_,rocker_r1,rocker_l1,rocker_l_,knob_l,knob_r;} rc;
} FS_RC_ctrl_t;
extern int huart3;
FS_RC_ctrl_t *FS_RemoteControlInit(int *);
uint8_t FS_RemoteControlIsOnline(void);
uint8_t FS_RemoteControlGetSnapshot(FS_RC_ctrl_t *);
#endif

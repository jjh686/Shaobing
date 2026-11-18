#ifndef RM_GIMBAL_PROTOCOL_H
#define RM_GIMBAL_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
#define RM_GIMBAL_FRAME_SIZE 46u
#define RM_ID_GIMBAL 0x22u
#define RM_GIMBAL_YAW_VALID 1u
#define RM_GIMBAL_PITCH_VALID 2u
#define RM_GIMBAL_CALIBRATED 4u
#define RM_GIMBAL_FAULT 8u
typedef struct {
 uint32_t uptime_ms,sample_ms,command_received_ms,session;
 uint16_t sequence,status,sample_age_ms;
 uint8_t route;
 float yaw,pitch,yaw_rate,pitch_rate;
} rm_gimbal_t;
int rm_encode_gimbal(const rm_gimbal_t*,uint8_t out[RM_GIMBAL_FRAME_SIZE]);
int rm_decode_gimbal(const uint8_t*,size_t,rm_gimbal_t*);
#ifdef __cplusplus
}
#endif
#endif

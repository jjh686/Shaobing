#ifndef RM_CHASSIS_COMM_PROTOCOL_H
#define RM_CHASSIS_COMM_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define RM_FRAME_SIZE 34
#define RM_PROTOCOL_VERSION 1
#define RM_ID_COMMAND 0x20
#define RM_ID_FEEDBACK 0x21
#define RM_FLAG_ENABLE 1u
#define RM_STATUS_READY 1u
#define RM_STATUS_ACTIVE 2u
#define RM_STATUS_ESTOP 4u
#define RM_STATUS_MEASURED 8u
#define RM_STATUS_TIMEOUT 16u
#define RM_STATUS_MANUAL 32u
#define RM_STATUS_ACK_VALID 64u
/* New proposed payload; not binary-compatible with existing PolarBear firmware.
 * vx/vy in m/s, wz in rad/s; ROS body convention x forward, y left, z up.
 * len includes uptime: 28. Explicit little-endian integers / IEEE754 binary32. */
typedef struct {uint32_t uptime_ms,session;uint16_t sequence,ttl_ms,flags;
 uint8_t route;float vx,vy,wz;} rm_command_t;
typedef struct {uint32_t uptime_ms,session;uint16_t sequence,status,fault_mask;
 uint8_t route;float vx,vy,wz;} rm_feedback_t;
typedef struct {uint8_t data[RM_FRAME_SIZE];size_t used;} rm_parser_t;
int rm_encode_command(const rm_command_t*,uint8_t out[RM_FRAME_SIZE]);
int rm_decode_command(const uint8_t*,size_t,rm_command_t*);
int rm_encode_feedback(const rm_feedback_t*,uint8_t out[RM_FRAME_SIZE]);
int rm_decode_feedback(const uint8_t*,size_t,rm_feedback_t*);
void rm_parser_init(rm_parser_t*);
int rm_parser_push(rm_parser_t*,uint8_t,uint8_t out[RM_FRAME_SIZE]);
#ifdef __cplusplus
}
#endif
#endif

/* [USB_SP 新增 01 开始] 新增固定29/43字节SP字段定义和编译期长度约束。 */
#ifndef SP_PROTOCOL_H
#define SP_PROTOCOL_H

#include <stddef.h>
#include <float.h>
#include <stdint.h>

#define SP_COMMAND_SIZE 29u
#define SP_STATUS_SIZE 43u

/* Wire units: rad, rad/s, rad/s^2; little endian IEEE754 float32. */
#pragma pack(push, 1)
typedef struct {
    uint8_t head[2];
    uint8_t mode; /* 0=release, 1=control, 2=control + legacy fire request */
    float yaw, yaw_vel, yaw_acc;
    float pitch, pitch_vel, pitch_acc;
    uint16_t crc16;
} SP_Command;

typedef struct {
    uint8_t head[2];
    uint8_t mode; /* 0=idle, 1=auto aim, 2=small buff, 3=big buff */
    float q[4];  /* raw IMU body -> IMU reference, w x y z */
    float yaw, yaw_vel, pitch, pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} SP_Status;
#pragma pack(pop)

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128, "SP needs IEEE754 float32");
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error SP wire structs require a little-endian target
#endif
_Static_assert(sizeof(SP_Command) == SP_COMMAND_SIZE, "SP command size");
_Static_assert(sizeof(SP_Status) == SP_STATUS_SIZE, "SP status size");

typedef void (*SP_CommandCallback)(const SP_Command *command, void *context);
typedef struct {
    uint8_t data[SP_COMMAND_SIZE];
    size_t used;
    uint32_t accepted, crc_errors, invalid_frames;
} SP_Parser;

uint16_t SP_CRC16(const uint8_t *data, size_t size);
void SP_ParserInit(SP_Parser *parser);
void SP_ParserFeed(SP_Parser *parser, const uint8_t *data, size_t size,
                   SP_CommandCallback callback, void *context);
void SP_PackStatus(const SP_Status *status, uint8_t out[SP_STATUS_SIZE]);

#endif
/* [USB_SP 新增 01 结束] */

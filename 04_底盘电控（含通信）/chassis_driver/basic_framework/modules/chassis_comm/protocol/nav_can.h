#ifndef NAV_CAN_H
#define NAV_CAN_H
#include "rm_chassis_comm/protocol.h"
#include <stdbool.h>
#define NC_COMMAND_BASE 0x330u
#define NC_FEEDBACK_BASE 0x338u
#define NC_VERSION 0x0100u
#define NC_GROUP_MS 10u
/* Finish or abort transmission before the peer's 10 ms assembly deadline. */
#define NC_TX_MS 8u
#define NC_MAX_TTL_MS 150u
#define NC_FAULT_LOCK 0x0001u
#define NC_FAULT_RC 0x0002u
#define NC_FAULT_MOTOR 0x0004u
#define NC_FAULT_UNITS 0x0008u
#define NC_FAULT_RX 0x0010u
typedef struct { uint8_t data[3][8]; uint16_t base; } nc_group_t;
typedef struct { nc_group_t group; uint8_t next,active; uint32_t started_ms; } nc_rx_t;
typedef struct { nc_group_t group; uint8_t next,active; uint32_t started_ms; } nc_tx_t;
uint16_t nc_crc(const uint8_t *,unsigned);
bool nc_command_group(const rm_command_t *,nc_group_t *);
bool nc_command_decode(const nc_group_t *,rm_command_t *);
bool nc_feedback_group(const rm_feedback_t *,nc_group_t *);
bool nc_feedback_decode(const nc_group_t *,rm_feedback_t *);
/* Unrelated CAN IDs leave receiver intact. Relevant malformed traffic aborts it. */
bool nc_rx_push(nc_rx_t *,uint16_t base,uint32_t id,bool extended,bool rtr,
                unsigned dlc,const uint8_t *,uint32_t now,nc_group_t *);
bool nc_tx_start(nc_tx_t *,const nc_group_t *,uint32_t now);
bool nc_tx_peek(nc_tx_t *,uint32_t now,uint16_t *id,uint8_t bytes[8]);
void nc_tx_commit(nc_tx_t *);
#endif

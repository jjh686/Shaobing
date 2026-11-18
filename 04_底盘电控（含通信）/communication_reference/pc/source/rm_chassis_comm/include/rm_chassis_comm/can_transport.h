#ifndef RM_CHASSIS_COMM_CAN_TRANSPORT_H
#define RM_CHASSIS_COMM_CAN_TRANSPORT_H
#include <stddef.h>
#include <stdint.h>
#include "rm_chassis_comm/protocol.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Proposed reserved IDs: verify against the actual vehicle CAN allocation. */
#define RM_CAN_COMMAND_ID 0x320u
#define RM_CAN_FEEDBACK_ID 0x321u
#define RM_CAN_FRAGMENT_COUNT 6u
#define RM_CAN_TRANSFER_TIMEOUT_MS 20u
/* Callback queues ONE standard 11-bit, DLC=8 data frame. 1=accepted, 0=busy/error.
 * Caller serializes every call to a context. No HAL call or lock is supplied. */
typedef int (*rm_can_send_fn)(uint16_t id, const uint8_t data[8], void *context);
typedef struct {
  uint16_t expected_id;
  uint8_t expected_type, transfer_id, next_index, active;
  uint32_t started_ms;
  uint8_t data[RM_FRAME_SIZE];
  uint32_t completed, rejected, expired;
} rm_can_rx_t;

void rm_can_rx_init(rm_can_rx_t *, uint16_t expected_id, uint8_t expected_type);
void rm_can_rx_expire(rm_can_rx_t *, uint32_t now_ms);
/* Returns 1 only for complete CRC/payload-valid route=2 frame of expected type.
 * Extended, RTR (pass dlc=0), unrelated-ID traffic cannot refresh a transfer.
 * A completed frame still needs the endpoint MCU replay/watchdog checks. */
int rm_can_rx_push(rm_can_rx_t *, uint32_t id, int is_extended, uint8_t dlc,
                   const uint8_t data[8], uint32_t now_ms, uint8_t out[RM_FRAME_SIZE]);
/* One bounded attempt, at most six callback invocations. BUSY aborts the transfer;
 * never continue its remaining fragments later. A NEW attempt consumes a NEW
 * transfer ID even if interrupted. No sequence/session/TTL field is rewritten. */
int rm_can_send_frame(uint16_t id, uint8_t *next_transfer_id,
                      const uint8_t frame[RM_FRAME_SIZE], rm_can_send_fn, void *context);
#ifdef __cplusplus
}
#endif
#endif

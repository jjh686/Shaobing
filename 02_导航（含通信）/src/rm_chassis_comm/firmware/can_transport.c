#include "rm_chassis_comm/can_transport.h"
#include <string.h>

static int valid_wire(uint16_t id, uint8_t type, const uint8_t *frame)
{
  if (id == RM_CAN_COMMAND_ID && type == RM_ID_COMMAND) {
    rm_command_t command;
    return rm_decode_command(frame, RM_FRAME_SIZE, &command) && command.route == 2;
  }
  if (id == RM_CAN_FEEDBACK_ID && type == RM_ID_FEEDBACK) {
    rm_feedback_t feedback;
    return rm_decode_feedback(frame, RM_FRAME_SIZE, &feedback) && feedback.route == 2;
  }
  return 0;
}

void rm_can_rx_init(rm_can_rx_t *rx, uint16_t id, uint8_t type)
{
  if (!rx) return;
  memset(rx, 0, sizeof(*rx));
  rx->expected_id = id;
  rx->expected_type = type;
}

void rm_can_rx_expire(rm_can_rx_t *rx, uint32_t now_ms)
{
  if (rx && rx->active && (uint32_t)(now_ms - rx->started_ms) >= RM_CAN_TRANSFER_TIMEOUT_MS) {
    rx->active = 0;
    rx->next_index = 0;
    ++rx->expired;
  }
}

static int reject(rm_can_rx_t *rx)
{
  rx->active = 0;
  rx->next_index = 0;
  ++rx->rejected;
  return 0;
}

int rm_can_rx_push(rm_can_rx_t *rx, uint32_t id, int extended, uint8_t dlc,
                   const uint8_t data[8], uint32_t now_ms, uint8_t out[RM_FRAME_SIZE])
{
  uint8_t index, last;
  size_t offset, length;
  if (!rx || !out) return 0;
  rm_can_rx_expire(rx, now_ms);
  if (extended || id > 0x7ffu || id != rx->expected_id) return 0;
  if (!data || dlc != 8) return reject(rx);
  index = data[1] & 0x7fu;
  last = data[1] & 0x80u;
  if (index >= RM_CAN_FRAGMENT_COUNT || (!!last != (index == 5))) return reject(rx);
  if (index == 0) {
    if (rx->active && rx->transfer_id == data[0]) return reject(rx);
    /* A new transfer may replace an incomplete older attempt. */
    if (rx->active) ++rx->rejected;
    rx->active = 1;
    rx->transfer_id = data[0];
    rx->started_ms = now_ms;
    rx->next_index = 0;
  }
  if (!rx->active || data[0] != rx->transfer_id || index != rx->next_index)
    return reject(rx);
  offset = (size_t)index * 6u;
  length = index == 5 ? 4u : 6u;
  if (index == 5 && (data[6] || data[7])) return reject(rx);
  memcpy(rx->data + offset, data + 2, length);
  ++rx->next_index;
  if (index != 5) return 0;
  rx->active = 0;
  rx->next_index = 0;
  if (!valid_wire(rx->expected_id, rx->expected_type, rx->data)) {
    ++rx->rejected;
    return 0;
  }
  memcpy(out, rx->data, RM_FRAME_SIZE);
  ++rx->completed;
  return 1;
}

int rm_can_send_frame(uint16_t id, uint8_t *next_transfer_id,
                      const uint8_t frame[RM_FRAME_SIZE], rm_can_send_fn send, void *context)
{
  uint8_t transfer_id, index;
  uint8_t type = id == RM_CAN_COMMAND_ID ? RM_ID_COMMAND : RM_ID_FEEDBACK;
  if (!frame || !next_transfer_id || !send || !valid_wire(id, type, frame)) return 0;
  transfer_id = (*next_transfer_id)++;
  for (index = 0; index < RM_CAN_FRAGMENT_COUNT; ++index) {
    uint8_t fragment[8] = {0};
    size_t length = index == 5 ? 4u : 6u;
    fragment[0] = transfer_id;
    fragment[1] = index | (index == 5 ? 0x80u : 0u);
    memcpy(fragment + 2, frame + (size_t)index * 6u, length);
    if (send(id, fragment, context) != 1) return 0;
  }
  return 1;
}

#include "gimbal_can.h"
#include <math.h>
#include <string.h>

static uint16_t sp_crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xffffu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (unsigned b = 0; b < 8; ++b)
      crc = (uint16_t)((crc >> 1) ^ ((crc & 1u) ? 0x8408u : 0u));
  }
  return crc;
}
static int sp_frame_valid(const uint8_t *data, size_t size)
{
  return data && (size == 29 || size == 43) && data[0] == 'S' && data[1] == 'P' &&
    sp_crc16(data, size - 2) == (uint16_t)(data[size-2] | ((uint16_t)data[size-1] << 8));
}
static float sp_float(const uint8_t *data)
{
  uint32_t value = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  float result;
  memcpy(&result, &value, sizeof(result));
  return result;
}
static int sp_command_valid(const uint8_t *data)
{
  if (data[2] > 2) return 0;
  for (size_t i = 3; i < 27; i += 4) if (!isfinite(sp_float(data+i))) return 0;
  return fabsf(sp_float(data+3)) <= 3.141593f && fabsf(sp_float(data+15)) <= 1.570797f;
}
static void usb_drop_byte(rm_gimbal_gateway_t *gateway)
{
  if (gateway->usb_used) {
    --gateway->usb_used;
    memmove(gateway->usb_data, gateway->usb_data+1, gateway->usb_used);
  }
}

void rm_gimbal_gateway_init(rm_gimbal_gateway_t *gateway, rm_can_send_fn can,
                             void *can_context, rm_gateway_usb_send_fn usb, void *usb_context)
{
  if (!gateway) return;
  memset(gateway, 0, sizeof(*gateway));
  rm_can_rx_init(&gateway->feedback_rx, RM_CAN_FEEDBACK_ID, RM_ID_FEEDBACK);
  gateway->send_can = can; gateway->can_context = can_context;
  gateway->send_usb = usb; gateway->usb_context = usb_context;
}

void rm_gateway_set_sp_handler(rm_gimbal_gateway_t *gateway,
                                rm_gateway_sp_command_fn handler, void *context)
{
  if (gateway) { gateway->sp_command = handler; gateway->sp_context = context; }
}

size_t rm_gimbal_gateway_usb_receive(rm_gimbal_gateway_t *gateway,
                                     const uint8_t *data, size_t length)
{
  size_t forwarded = 0;
  if (!gateway || (!data && length)) return 0;
  for (size_t i = 0; i < length; ++i) {
    if (gateway->usb_used >= sizeof(gateway->usb_data)) usb_drop_byte(gateway);
    gateway->usb_data[gateway->usb_used++] = data[i];
    while (gateway->usb_used) {
      uint8_t *bytes = gateway->usb_data;
      if (bytes[0] == 0x5a) {
        rm_parser_t check;
        uint8_t frame[RM_FRAME_SIZE];
        int complete = 0;
        size_t prefix = gateway->usb_used < 4 ? gateway->usb_used : 4;
        rm_parser_init(&check);
        for (size_t j = 0; j < prefix; ++j) rm_parser_push(&check, bytes[j], frame);
        if (check.used != prefix) { usb_drop_byte(gateway); continue; }
        if (gateway->usb_used < RM_FRAME_SIZE) break;
        rm_parser_init(&check);
        for (size_t j = 0; j < RM_FRAME_SIZE; ++j)
          complete = rm_parser_push(&check, bytes[j], frame);
        if (!complete) { usb_drop_byte(gateway); continue; }
        /* Consume a whole CRC-valid new frame even if its business payload is
         * rejected: its data bytes must never become another SP command. */
        gateway->usb_used = 0;
        rm_command_t command;
        if (rm_decode_command(frame, RM_FRAME_SIZE, &command) && command.route == 2) {
          if (rm_can_send_frame(RM_CAN_COMMAND_ID, &gateway->command_transfer, frame,
                                gateway->send_can, gateway->can_context)) {
            ++gateway->command_forwarded; ++forwarded;
          } else ++gateway->dropped;
        }
      } else if (bytes[0] == 'S') {
        if (gateway->usb_used < 2) break;
        if (bytes[1] != 'P') { usb_drop_byte(gateway); continue; }
        if (gateway->usb_used < 29) break;
        if (!sp_frame_valid(bytes,29)) { usb_drop_byte(gateway); continue; }
        if (sp_command_valid(bytes) && gateway->sp_command) {
          gateway->sp_command(bytes, gateway->sp_context);
          ++gateway->sp_commands;
        }
        gateway->usb_used = 0;
      } else usb_drop_byte(gateway);
    }
  }
  return forwarded;
}

int rm_gimbal_gateway_send_sp_status(rm_gimbal_gateway_t *gateway, const uint8_t frame[43])
{
  if (!gateway || !sp_frame_valid(frame,43) || frame[2] > 3) return 0;
  if (!gateway->send_usb || gateway->send_usb(frame,43,gateway->usb_context) != 1) {
    ++gateway->dropped; return 0;
  }
  ++gateway->sp_status_forwarded;
  return 1;
}

int rm_gimbal_gateway_can_receive(rm_gimbal_gateway_t *gateway, uint32_t id,
                                  int extended, uint8_t dlc, const uint8_t data[8], uint32_t now_ms)
{
  uint8_t frame[RM_FRAME_SIZE];
  if (!gateway || !rm_can_rx_push(&gateway->feedback_rx, id, extended, dlc,
                                  data, now_ms, frame)) return 0;
  if (!gateway->send_usb || gateway->send_usb(frame, RM_FRAME_SIZE, gateway->usb_context) != 1) {
    ++gateway->dropped; return 0;
  }
  ++gateway->feedback_forwarded;
  return 1;
}

void rm_gimbal_gateway_poll(rm_gimbal_gateway_t *gateway, uint32_t now_ms)
{
  if (gateway) rm_can_rx_expire(&gateway->feedback_rx, now_ms);
}

void rm_chassis_can_init(rm_chassis_can_t *link, rm_can_send_fn send_can,
                          void *can_context, rm_chassis_apply_fn apply, void *apply_context)
{
  if (!link) return;
  memset(link, 0, sizeof(*link));
  rm_mcu_init(&link->mcu, 2);
  rm_can_rx_init(&link->command_rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  link->send_can = send_can; link->can_context = can_context;
  link->apply = apply; link->apply_context = apply_context;
}

void rm_chassis_can_set_safety(rm_chassis_can_t *link, bool ready, bool estop,
                               bool manual, uint16_t fault)
{
  if (link) rm_mcu_set_safety(&link->mcu, ready && link->apply != NULL, estop, manual, fault);
}

int rm_chassis_can_receive(rm_chassis_can_t *link, uint32_t id, int extended,
                           uint8_t dlc, const uint8_t data[8], uint32_t now_ms)
{
  uint8_t frame[RM_FRAME_SIZE];
  if (!link || !rm_can_rx_push(&link->command_rx, id, extended, dlc, data, now_ms, frame))
    return 0;
  if (!rm_mcu_receive(&link->mcu, frame, RM_FRAME_SIZE, now_ms)) return 0;
  ++link->received;
  return 1;
}

void rm_chassis_can_poll(rm_chassis_can_t *link, uint32_t now_ms, bool measured_valid,
                         float vx, float vy, float wz)
{
  rm_output_t output = {0};
  rm_feedback_t feedback;
  uint8_t frame[RM_FRAME_SIZE];
  if (!link) return;
  rm_can_rx_expire(&link->command_rx, now_ms);
  rm_mcu_get_output(&link->mcu, now_ms, &output);
  if (link->apply) link->apply(&output, link->apply_context);
  if (link->feedback_started && (uint32_t)(now_ms - link->feedback_ms) < 20u) return;
  link->feedback_started = 1; link->feedback_ms = now_ms;
  rm_mcu_feedback(&link->mcu, now_ms, measured_valid, vx, vy, wz, &feedback);
  if (!rm_encode_feedback(&feedback, frame)) return;
  if (!rm_can_send_frame(RM_CAN_FEEDBACK_ID, &link->feedback_transfer, frame,
                         link->send_can, link->can_context)) ++link->feedback_dropped;
}

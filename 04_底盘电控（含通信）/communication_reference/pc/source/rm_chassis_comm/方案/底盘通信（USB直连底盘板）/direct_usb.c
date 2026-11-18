#include "direct_usb.h"
#include <string.h>

void rm_direct_usb_init(rm_direct_usb_t *link, rm_direct_usb_send_fn send_usb,
                         void *usb_context, rm_direct_apply_fn apply, void *apply_context)
{
  if (!link) return;
  memset(link, 0, sizeof(*link));
  rm_parser_init(&link->parser);
  rm_mcu_init(&link->mcu, 1);
  link->send_usb = send_usb; link->usb_context = usb_context;
  link->apply = apply; link->apply_context = apply_context;
}

void rm_direct_usb_set_safety(rm_direct_usb_t *link, bool ready, bool estop,
                              bool manual, uint16_t fault)
{
  if (link) rm_mcu_set_safety(&link->mcu, ready && link->apply != NULL, estop, manual, fault);
}

size_t rm_direct_usb_receive(rm_direct_usb_t *link, const uint8_t *data,
                             size_t length, uint32_t now_ms)
{
  size_t accepted = 0;
  uint8_t frame[RM_FRAME_SIZE];
  if (!link || (!data && length)) return 0;
  for (size_t i = 0; i < length; ++i) {
    if (rm_parser_push(&link->parser, data[i], frame) &&
        rm_mcu_receive(&link->mcu, frame, RM_FRAME_SIZE, now_ms)) {
      ++accepted; ++link->received;
    }
  }
  return accepted;
}

void rm_direct_usb_poll(rm_direct_usb_t *link, uint32_t now_ms, bool measured_valid,
                        float vx, float vy, float wz)
{
  rm_output_t output = {0};
  rm_feedback_t feedback;
  uint8_t frame[RM_FRAME_SIZE];
  if (!link) return;
  rm_mcu_get_output(&link->mcu, now_ms, &output);
  if (link->apply) link->apply(&output, link->apply_context);
  if (link->feedback_started && (uint32_t)(now_ms - link->feedback_ms) < 20u) return;
  link->feedback_started = 1; link->feedback_ms = now_ms;
  rm_mcu_feedback(&link->mcu, now_ms, measured_valid, vx, vy, wz, &feedback);
  if (!rm_encode_feedback(&feedback, frame)) return;
  if (link->send_usb && link->send_usb(frame, RM_FRAME_SIZE, link->usb_context) != 1)
    ++link->feedback_busy;
}

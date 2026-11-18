#ifndef RM_CHASSIS_DIRECT_USB_H
#define RM_CHASSIS_DIRECT_USB_H
#include "rm_chassis_comm/mcu_link.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Nonblocking callbacks: send returns 1 only after copying all bytes to a TX
 * owner; 0 means busy/error. apply must honor enabled=false as chassis stop. */
typedef int (*rm_direct_usb_send_fn)(const uint8_t *, size_t, void *);
typedef void (*rm_direct_apply_fn)(const rm_output_t *, void *);
typedef struct {
  rm_parser_t parser;
  rm_mcu_link_t mcu;
  rm_direct_usb_send_fn send_usb;
  rm_direct_apply_fn apply;
  void *usb_context, *apply_context;
  uint32_t feedback_ms, feedback_busy, received;
  uint8_t feedback_started;
} rm_direct_usb_t;

/* Init leaves safety NOT ready. External hardware/remote/mode arbitration must
 * explicitly call set_safety. All functions on one instance have one owner. */
void rm_direct_usb_init(rm_direct_usb_t *, rm_direct_usb_send_fn, void *usb_context,
                         rm_direct_apply_fn, void *apply_context);
void rm_direct_usb_set_safety(rm_direct_usb_t *, bool ready, bool estop,
                              bool manual, uint16_t fault_mask);
size_t rm_direct_usb_receive(rm_direct_usb_t *, const uint8_t *, size_t, uint32_t now_ms);
/* Call from the regular control task, including when no USB packets arrive.
 * Measurement inputs must be ACTUAL fresh chassis velocity; false if unavailable.
 * No fabricated wheel odometry or command-as-measurement is emitted. */
void rm_direct_usb_poll(rm_direct_usb_t *, uint32_t now_ms, bool measured_valid,
                        float measured_vx, float measured_vy, float measured_wz);
#ifdef __cplusplus
}
#endif
#endif

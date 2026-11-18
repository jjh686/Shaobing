#ifndef RM_CHASSIS_GIMBAL_CAN_H
#define RM_CHASSIS_GIMBAL_CAN_H
#include "rm_chassis_comm/can_transport.h"
#include "rm_chassis_comm/mcu_link.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef int (*rm_gateway_usb_send_fn)(const uint8_t *, size_t, void *);
typedef void (*rm_chassis_apply_fn)(const rm_output_t *, void *);
typedef void (*rm_gateway_sp_command_fn)(const uint8_t frame[29], void *);
typedef struct {
  /* One leading-magic demultiplexer; never two parallel payload parsers. */
  uint8_t usb_data[RM_FRAME_SIZE];
  size_t usb_used;
  rm_can_rx_t feedback_rx;
  uint8_t command_transfer;
  rm_can_send_fn send_can;
  rm_gateway_usb_send_fn send_usb;
  void *can_context, *usb_context;
  uint32_t command_forwarded, feedback_forwarded, dropped;
  rm_gateway_sp_command_fn sp_command;
  void *sp_context;
  uint32_t sp_commands, sp_status_forwarded;
} rm_gimbal_gateway_t;

/* Gimbal role only forwards intact route=2 packets. It has no mcu_link and
 * cannot create a chassis READY, ACK, motion measurement or feedback frame. */
void rm_gimbal_gateway_init(rm_gimbal_gateway_t *, rm_can_send_fn, void *can_context,
                             rm_gateway_usb_send_fn, void *usb_context);
/* Optional complete valid SP command handoff to the existing gimbal owner.
 * Callback must copy/use the 29 bytes before return; never route to Seasky. */
void rm_gateway_set_sp_handler(rm_gimbal_gateway_t *, rm_gateway_sp_command_fn, void *);
size_t rm_gimbal_gateway_usb_receive(rm_gimbal_gateway_t *, const uint8_t *, size_t);
int rm_gimbal_gateway_can_receive(rm_gimbal_gateway_t *, uint32_t id, int extended,
                                  uint8_t dlc, const uint8_t[8], uint32_t now_ms);
void rm_gimbal_gateway_poll(rm_gimbal_gateway_t *, uint32_t now_ms);
/* Existing SP status producer uses the SAME USB TX owner as chassis feedback. */
int rm_gimbal_gateway_send_sp_status(rm_gimbal_gateway_t *, const uint8_t frame[43]);

typedef struct {
  rm_mcu_link_t mcu;
  rm_can_rx_t command_rx;
  rm_can_send_fn send_can;
  rm_chassis_apply_fn apply;
  void *can_context, *apply_context;
  uint8_t feedback_transfer, feedback_started;
  uint32_t feedback_ms, feedback_dropped, received;
} rm_chassis_can_t;
void rm_chassis_can_init(rm_chassis_can_t *, rm_can_send_fn, void *can_context,
                          rm_chassis_apply_fn, void *apply_context);
void rm_chassis_can_set_safety(rm_chassis_can_t *, bool ready, bool estop,
                               bool manual, uint16_t fault_mask);
int rm_chassis_can_receive(rm_chassis_can_t *, uint32_t id, int extended,
                           uint8_t dlc, const uint8_t[8], uint32_t now_ms);
/* Chassis task owns its safety gate and sends only its genuine MCU feedback. */
void rm_chassis_can_poll(rm_chassis_can_t *, uint32_t now_ms, bool measured_valid,
                         float measured_vx, float measured_vy, float measured_wz);
#ifdef __cplusplus
}
#endif
#endif

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "rm_chassis_comm/can_transport.h"
#include "../方案/底盘通信（USB直连底盘板）/direct_usb.h"
#include "../方案/底盘通信（USB经云台CAN转发）/gimbal_can.h"

typedef struct {
  uint8_t packets[6][8];
  unsigned accepted, calls, busy_at;
  uint16_t id;
} sink_t;
static int queue(uint16_t id, const uint8_t data[8], void *context)
{
  sink_t *sink = (sink_t *)context;
  ++sink->calls;
  if (sink->busy_at && sink->calls == sink->busy_at) return 0;
  assert(sink->accepted < 6);
  sink->id = id;
  memcpy(sink->packets[sink->accepted++], data, 8);
  return 1;
}
static sink_t packets(const uint8_t frame[RM_FRAME_SIZE], uint16_t id, uint8_t transfer)
{
  sink_t sink = {0};
  assert(rm_can_send_frame(id, &transfer, frame, queue, &sink));
  assert(sink.accepted == 6 && sink.calls == 6);
  return sink;
}
static unsigned deliver(rm_can_rx_t *rx, const sink_t *sink, uint32_t start, uint8_t *out)
{
  unsigned got = 0;
  for (unsigned i = 0; i < sink->accepted; ++i)
    got += rm_can_rx_push(rx, sink->id, 0, 8, sink->packets[i], start + i, out);
  return got;
}
typedef struct {
  uint8_t frame[43]; size_t length;
  unsigned calls, sp_calls; int busy;
  rm_output_t applied;
} io_t;
static int usb_copy(const uint8_t *data, size_t length, void *context)
{
  io_t *io = (io_t *)context;
  if (io->busy) return 0;
  assert(length <= sizeof(io->frame));
  memcpy(io->frame,data,length); io->length=length; ++io->calls;
  return 1;
}
static void apply(const rm_output_t *output, void *context)
{
  ((io_t *)context)->applied = *output;
}
static void sp_command(const uint8_t frame[29], void *context)
{
  assert(frame[0]=='S' && frame[1]=='P');
  ++((io_t *)context)->sp_calls;
}
static uint16_t sp_crc(const uint8_t *data, size_t n)
{
  uint16_t crc=0xffffu;
  for(size_t i=0;i<n;++i){
    crc^=data[i];
    for(unsigned bit=0;bit<8;++bit) crc=(uint16_t)((crc>>1)^((crc&1u)?0x8408u:0u));
  }
  return crc;
}
static void finish_sp(uint8_t *data, size_t n)
{
  uint16_t crc=sp_crc(data,n-2);
  data[n-2]=(uint8_t)crc; data[n-1]=(uint8_t)(crc>>8);
}
static void test_direct(void)
{
  rm_direct_usb_t direct;
  io_t io={0};
  uint8_t frame[RM_FRAME_SIZE];
  rm_command_t command={0};
  rm_feedback_t status;
  command.route=1; command.session=55;
  rm_direct_usb_init(&direct,usb_copy,&io,NULL,NULL);
  rm_direct_usb_set_safety(&direct,true,false,false,0);
  assert(!direct.mcu.ready);  /* No registered actuator adapter => never ready. */
  rm_direct_usb_init(&direct,usb_copy,&io,apply,&io);
  rm_direct_usb_poll(&direct,0,false,1,2,3);
  assert(rm_decode_feedback(io.frame,io.length,&status));
  assert(!(status.status&(RM_STATUS_READY|RM_STATUS_ACK_VALID|RM_STATUS_MEASURED)));
  rm_direct_usb_set_safety(&direct,true,false,false,0);
  assert(rm_encode_command(&command,frame));
  for(size_t i=0;i<RM_FRAME_SIZE;++i)
    assert(rm_direct_usb_receive(&direct,frame+i,1,10)==(i+1==RM_FRAME_SIZE));
  rm_direct_usb_poll(&direct,20,false,0,0,0);
  assert(rm_decode_feedback(io.frame,io.length,&status));
  assert(status.route==1 && status.session==55 && (status.status&RM_STATUS_ACK_VALID));
  command.sequence=1; command.flags=RM_FLAG_ENABLE; command.ttl_ms=100; command.vx=.2f;
  assert(rm_encode_command(&command,frame));
  assert(rm_direct_usb_receive(&direct,frame,RM_FRAME_SIZE,21)==1);
  rm_direct_usb_poll(&direct,40,false,9,9,9);
  assert(io.applied.enabled && io.applied.vx==.2f);
  assert(rm_decode_feedback(io.frame,io.length,&status));
  assert((status.status&RM_STATUS_ACTIVE) && !(status.status&RM_STATUS_MEASURED));
  assert(status.vx==0 && status.vy==0 && status.wz==0);
  rm_direct_usb_poll(&direct,121,false,0,0,0);
  assert(!io.applied.enabled);
  assert(rm_decode_feedback(io.frame,io.length,&status));
  assert(status.status&RM_STATUS_TIMEOUT);
  assert(!rm_direct_usb_receive(&direct,frame,RM_FRAME_SIZE,122));
  command.route=2; command.flags=0; command.ttl_ms=0; command.vx=0;
  assert(rm_encode_command(&command,frame));
  assert(!rm_direct_usb_receive(&direct,frame,RM_FRAME_SIZE,123));
  io.busy=1;
  rm_direct_usb_poll(&direct,150,false,0,0,0);
  assert(direct.feedback_busy==1);
}
static void test_gateway_and_chassis(void)
{
  rm_gimbal_gateway_t gateway;
  rm_chassis_can_t chassis;
  sink_t commands={0}, statuses={0};
  io_t io={0}, motor={0};
  uint8_t frame[RM_FRAME_SIZE], sp[29]={0}, mixed[RM_FRAME_SIZE+58], sp_status[43]={0};
  rm_command_t command={0}; rm_feedback_t status;
  command.route=2; command.session=77;
  rm_gimbal_gateway_init(&gateway,queue,&commands,usb_copy,&io);
  rm_chassis_can_init(&chassis,queue,&statuses,NULL,NULL);
  rm_chassis_can_set_safety(&chassis,true,false,false,0);
  assert(!chassis.mcu.ready);
  rm_chassis_can_init(&chassis,queue,&statuses,apply,&motor);
  rm_chassis_can_set_safety(&chassis,true,false,false,0);
  assert(rm_encode_command(&command,frame));
  assert(rm_gimbal_gateway_usb_receive(&gateway,frame,RM_FRAME_SIZE)==1);
  assert(!io.calls); /* Gateway forwarding is never an invented MCU ACK. */
  for(unsigned i=0;i<6;++i)
    assert(rm_chassis_can_receive(&chassis,commands.id,0,8,commands.packets[i],100+i)==(i==5));
  rm_chassis_can_poll(&chassis,106,false,0,0,0);
  for(unsigned i=0;i<6;++i)
    assert(rm_gimbal_gateway_can_receive(&gateway,statuses.id,0,8,statuses.packets[i],107+i)==(i==5));
  assert(io.calls==1 && rm_decode_feedback(io.frame,io.length,&status));
  assert(status.session==77 && status.route==2 && (status.status&RM_STATUS_ACK_VALID));
  assert(!motor.applied.enabled);

  commands.accepted=commands.calls=0;
  command.sequence=1; command.flags=RM_FLAG_ENABLE; command.ttl_ms=100; command.vx=.3f;
  assert(rm_encode_command(&command,frame));
  assert(rm_gimbal_gateway_usb_receive(&gateway,frame,RM_FRAME_SIZE)==1);
  for(unsigned i=0;i<6;++i)
    rm_chassis_can_receive(&chassis,commands.id,0,8,commands.packets[i],120+i);
  uint32_t accepted_ms=chassis.mcu.received_ms;
  commands.accepted=commands.calls=0;
  command.sequence=2;
  assert(rm_encode_command(&command,frame));
  assert(rm_gimbal_gateway_usb_receive(&gateway,frame,RM_FRAME_SIZE)==1);
  for(unsigned i=0;i<5;++i)
    assert(!rm_chassis_can_receive(&chassis,commands.id,0,8,commands.packets[i],130+i));
  assert(chassis.mcu.received_ms==accepted_ms);
  statuses.accepted=statuses.calls=0;
  rm_chassis_can_poll(&chassis,151,false,0,0,0);
  assert(!chassis.command_rx.active && motor.applied.enabled);
  statuses.accepted=statuses.calls=0;
  rm_chassis_can_poll(&chassis,225,false,0,0,0);
  assert(!motor.applied.enabled); /* Missing transfer did not postpone original TTL. */

  /* The shared USB owner consumes complete frames before looking for other magic. */
  rm_gateway_set_sp_handler(&gateway,sp_command,&io);
  sp[0]='S'; sp[1]='P'; sp[2]=1;
  memcpy(sp+7,frame,4); /* New-frame magic inside a valid finite SP velocity field. */
  finish_sp(sp,sizeof(sp));
  command.flags=0; command.ttl_ms=0; command.vx=0;
  command.session=0x00015053u; /* SP magic inside a valid chassis payload. */
  assert(rm_encode_command(&command,frame));
  memcpy(mixed,sp,29); memcpy(mixed+29,frame,RM_FRAME_SIZE); memcpy(mixed+29+RM_FRAME_SIZE,sp,29);
  commands.accepted=commands.calls=0;
  for(size_t i=0;i<sizeof(mixed);++i) rm_gimbal_gateway_usb_receive(&gateway,mixed+i,1);
  assert(io.sp_calls==2 && commands.accepted==6);
  rm_can_rx_t rx; uint8_t restored[RM_FRAME_SIZE];
  rm_can_rx_init(&rx,RM_CAN_COMMAND_ID,RM_ID_COMMAND);
  assert(deliver(&rx,&commands,500,restored)==1 && !memcmp(restored,frame,RM_FRAME_SIZE));
  sp[28]^=1;
  commands.accepted=commands.calls=0;
  rm_gimbal_gateway_usb_receive(&gateway,sp,29);
  assert(rm_gimbal_gateway_usb_receive(&gateway,frame,RM_FRAME_SIZE)==1);
  assert(io.sp_calls==2);
  sp_status[0]='S'; sp_status[1]='P'; sp_status[2]=1;
  finish_sp(sp_status,sizeof(sp_status));
  assert(rm_gimbal_gateway_send_sp_status(&gateway,sp_status));
  assert(io.length==43 && !memcmp(io.frame,sp_status,43));
  io.busy=1;
  assert(!rm_gimbal_gateway_send_sp_status(&gateway,sp_status));
}
int main(void)
{
  uint8_t frame[RM_FRAME_SIZE], out[RM_FRAME_SIZE], transfer = 255;
  rm_command_t command = {0};
  command.route = 2; command.session = 19; command.sequence = 4;
  command.ttl_ms = 100; command.flags = RM_FLAG_ENABLE;
  assert(rm_encode_command(&command, frame));
  sink_t good = packets(frame, RM_CAN_COMMAND_ID, 7);
  assert(good.packets[5][1] == 0x85 && !good.packets[5][6] && !good.packets[5][7]);
  rm_can_rx_t rx;
  rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  assert(deliver(&rx, &good, 100, out) == 1);
  assert(!memcmp(frame, out, RM_FRAME_SIZE));

  /* Wrong CAN ID, IDE, DLC and packet type/route never yield a command. */
  rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  assert(!rm_can_rx_push(&rx, 0x319, 0, 8, good.packets[0], 0, out));
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 1, 8, good.packets[0], 0, out));
  assert(!rm_can_rx_push(&rx, 0x1320, 0, 8, good.packets[0], 0, out));
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 7, good.packets[0], 0, out));
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[1], 0, out));
  assert(!rx.completed);
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[0], 1, out));
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[2], 2, out));
  for (unsigned i = 3; i < 6; ++i)
    assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[i], i, out));
  assert(deliver(&rx, &good, 100, out) == 1);

  /* Duplicate, mismatched transfer ID, wrong last marker, nonzero padding and CRC. */
  for (unsigned error = 0; error < 6; ++error) {
    sink_t bad = good;
    rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
    if (error == 0) bad.packets[2][0]++;
    if (error == 1) bad.packets[2][1] |= 0x80;
    if (error == 2) bad.packets[5][1] = 5;
    if (error == 3) bad.packets[5][6] = 1;
    if (error == 4) bad.packets[3][4] ^= 0x40;
    if (error == 5) memcpy(bad.packets[2], bad.packets[1], 8);
    assert(deliver(&rx, &bad, 100, out) == 0);
    assert(deliver(&rx, &good, 200, out) == 1);
  }
  rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[0], 100, out));
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[0], 101, out));
  assert(!rx.active);

  /* 20 ms is a TOTAL deadline, not refreshed by individual fragments. */
  rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[0], 100, out));
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[1], 119, out));
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[2], 120, out));
  assert(rx.expired == 1 && !rx.active);
  assert(deliver(&rx, &good, UINT32_MAX - 2u, out) == 1);

  /* Another valid transfer replaces an incomplete attempt; transfer IDs wrap. */
  sink_t next = packets(frame, RM_CAN_COMMAND_ID, 8);
  rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  assert(!rm_can_rx_push(&rx, RM_CAN_COMMAND_ID, 0, 8, good.packets[0], 10, out));
  assert(deliver(&rx, &next, 11, out) == 1);
  sink_t wrapped = {0};
  assert(rm_can_send_frame(RM_CAN_COMMAND_ID, &transfer, frame, queue, &wrapped));
  assert(transfer == 0 && wrapped.packets[0][0] == 255);
  wrapped.accepted = wrapped.calls = 0;
  assert(rm_can_send_frame(RM_CAN_COMMAND_ID, &transfer, frame, queue, &wrapped));
  assert(transfer == 1 && wrapped.packets[0][0] == 0);

  /* A busy queue aborts immediately; no tail retries and the next attempt is new. */
  for (unsigned at = 1; at <= 6; ++at) {
    sink_t busy = {0}; busy.busy_at = at;
    assert(!rm_can_send_frame(RM_CAN_COMMAND_ID, &transfer, frame, queue, &busy));
    assert(busy.calls == at && busy.accepted == at - 1);
    rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
    assert(!deliver(&rx, &busy, 10, out));
    rm_can_rx_expire(&rx, 30);
    assert(!rx.active && !rx.completed);
  }

  command.route = 1;
  assert(rm_encode_command(&command, frame));
  sink_t discarded = {0};
  assert(!rm_can_send_frame(RM_CAN_COMMAND_ID, &transfer, frame, queue, &discarded));
  assert(!discarded.calls);
  /* Put a valid route=1 wire image into otherwise correct route=2 transport. */
  for (unsigned i = 0; i < 6; ++i)
    memcpy(good.packets[i] + 2, frame + 6u * i, i == 5 ? 4u : 6u);
  rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  assert(!deliver(&rx, &good, 100, out));

  rm_feedback_t feedback = {0}; feedback.route = 2;
  assert(rm_encode_feedback(&feedback, frame));
  good = packets(frame, RM_CAN_FEEDBACK_ID, 31);
  rm_can_rx_init(&rx, RM_CAN_FEEDBACK_ID, RM_ID_FEEDBACK);
  assert(deliver(&rx, &good, 200, out) == 1);
  assert(!memcmp(frame, out, RM_FRAME_SIZE));
  rm_can_rx_init(&rx, RM_CAN_COMMAND_ID, RM_ID_COMMAND);
  good.id = RM_CAN_COMMAND_ID;
  assert(!deliver(&rx, &good, 300, out));
  test_direct();
  test_gateway_and_chassis();
  return 0;
}

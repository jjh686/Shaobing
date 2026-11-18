#ifdef NDEBUG
#undef NDEBUG
#endif
#include "rm_chassis_comm/stream_mux.hpp"
#include <cassert>
#include <vector>
using namespace rm_chassis_comm;
template<size_t N>std::array<uint8_t,N> sp() {
  std::array<uint8_t,N> frame{};frame[0]='S';frame[1]='P';
  const auto crc=sp_crc(frame.data(),N-2);frame[N-2]=uint8_t(crc);frame[N-1]=uint8_t(crc>>8);return frame;
}
int main() {
  rm_feedback_t feedback{};feedback.route=2;feedback.session=1;
  feedback.status=RM_STATUS_READY|RM_STATUS_ACK_VALID;
  feedback.uptime_ms=0x5053; // The SP magic occurs inside a valid chassis payload.
  std::array<uint8_t,RM_FRAME_SIZE> chassis{};assert(rm_encode_feedback(&feedback,chassis.data()));
  auto vision=sp<43>();auto command=sp<29>();
  StreamMux<43> mux;unsigned c=0,s=0;
  auto push=[&](const auto & bytes,bool enable=true) {
    for(auto byte:bytes)mux.push(byte,[&](const rm_feedback_t & f){assert(f.session==1);++c;},
      [&](const uint8_t * p,size_t n){assert(n==43&&sp_valid(p,n));++s;},enable);
  };
  push(std::vector<uint8_t>{0,1,'S',0,0x5a,0,0,0});
  push(vision);push(chassis);push(vision);push(chassis);
  assert(c==2&&s==2);
  auto broken=vision;broken[15]^=1;push(broken);push(chassis);
  assert(c==3&&s==2);
  mux.reset();push(vision,false);push(chassis,false);assert(c==4&&s==2);
  SpCommandParser parser;std::array<uint8_t,29> out{};unsigned commands=0;
  for(auto b:std::vector<uint8_t>{0,1,'S',0})commands+=parser.push(b,out);
  for(auto b:command)commands+=parser.push(b,out);
  assert(commands==1&&out==command);
  command[20]^=1;for(auto b:command)commands+=parser.push(b,out);
  assert(commands==1);
  command=sp<29>();for(auto b:command)commands+=parser.push(b,out);
  assert(commands==2);
  rm_gimbal_t joint{};joint.route=2;joint.session=1;joint.status=7;
  joint.uptime_ms=0x5053;joint.sample_ms=joint.uptime_ms;joint.command_received_ms=joint.uptime_ms;
  std::array<uint8_t,RM_GIMBAL_FRAME_SIZE> joint_bytes{};
  assert(rm_encode_gimbal(&joint,joint_bytes.data()));
  unsigned g=0;StreamMux<43> three;
  auto mixed=[&](const auto & bytes) {
    for(auto byte:bytes)three.push(byte,[&](const rm_feedback_t &){++c;},
      [&](const uint8_t * p,size_t n){assert(n==43&&sp_valid(p,n));++s;},
      [&](const rm_gimbal_t & decoded){assert(decoded.uptime_ms==0x5053);++g;},true);
  };
  mixed(joint_bytes);mixed(vision);mixed(chassis);assert(g==1&&c==5&&s==3);
  joint_bytes[28]^=1;mixed(joint_bytes);mixed(vision);assert(g==1&&s==4);
}

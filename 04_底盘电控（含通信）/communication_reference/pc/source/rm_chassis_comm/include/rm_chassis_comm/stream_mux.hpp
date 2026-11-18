// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "rm_chassis_comm/protocol.h"
#include "rm_chassis_comm/gimbal_protocol.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rm_chassis_comm {
inline uint16_t sp_crc(const uint8_t * bytes, size_t size) {
  uint16_t crc=0xffff;
  for(size_t i=0;i<size;++i) {
    crc^=bytes[i];
    for(unsigned j=0;j<8;++j) crc=(crc&1)?uint16_t((crc>>1)^0x8408):uint16_t(crc>>1);
  }
  return crc;
}
inline bool sp_valid(const uint8_t * bytes,size_t n) {
  return n>=4 && bytes[0]=='S' && bytes[1]=='P' &&
    sp_crc(bytes,n-2)==uint16_t(bytes[n-2]|uint16_t(bytes[n-1])<<8);
}

// One physical stream owner: consume a validated whole frame before looking for
// another magic. Thus a payload containing SP or 5A cannot be dispatched twice.
template<size_t SpSize> class StreamMux {
public:
  void reset() { size_=0; }
  template<class OnChassis,class OnSp>
  void push(uint8_t byte,OnChassis chassis,OnSp sp,bool accept_sp) {
    push(byte,chassis,sp,[](const rm_gimbal_t &){},accept_sp);
  }
  template<class OnChassis,class OnSp,class OnGimbal>
  void push(uint8_t byte,OnChassis chassis,OnSp sp,OnGimbal gimbal,bool accept_sp) {
    if(size_==buffer_.size()) discard();
    buffer_[size_++]=byte;
    while(size_) {
      if(buffer_[0]==0x5a) {
        if(size_<4) return;
        if(buffer_[1]==28 && buffer_[2]==RM_ID_FEEDBACK) {
          if(size_<RM_FRAME_SIZE)return;
          rm_feedback_t decoded{};
          if(rm_decode_feedback(buffer_.data(),RM_FRAME_SIZE,&decoded)) {
            chassis(decoded);consume(RM_FRAME_SIZE);continue;
          }
        } else if(buffer_[1]==40 && buffer_[2]==RM_ID_GIMBAL) {
          if(size_<RM_GIMBAL_FRAME_SIZE)return;
          rm_gimbal_t decoded{};
          if(rm_decode_gimbal(buffer_.data(),RM_GIMBAL_FRAME_SIZE,&decoded)) {
            gimbal(decoded);consume(RM_GIMBAL_FRAME_SIZE);continue;
          }
        }
      } else if(accept_sp && buffer_[0]=='S') {
        if(size_<2) return;
        if(buffer_[1]!='P') {discard();continue;}
        if(size_<SpSize) return;
        if(sp_valid(buffer_.data(),SpSize)) {
          sp(buffer_.data(),SpSize);consume(SpSize);continue;
        }
      }
      discard();
    }
  }
private:
  void consume(size_t n) {
    size_-=n;
    if(size_) std::memmove(buffer_.data(),buffer_.data()+n,size_);
  }
  void discard() { consume(1); }
  std::array<uint8_t,SpSize+RM_FRAME_SIZE+RM_GIMBAL_FRAME_SIZE> buffer_{};
  size_t size_=0;
};

class SpCommandParser {
public:
  void reset() {size_=0;}
  bool push(uint8_t byte,std::array<uint8_t,29> & out) {
    if(size_==buffer_.size()) discard();
    buffer_[size_++]=byte;
    while(size_) {
      if(buffer_[0]!='S') {discard();continue;}
      if(size_<2) return false;
      if(buffer_[1]!='P') {discard();continue;}
      if(size_<buffer_.size()) return false;
      if(sp_valid(buffer_.data(),buffer_.size())) {out=buffer_;size_=0;return true;}
      discard();
    }
    return false;
  }
private:
  void discard() {--size_;if(size_)std::memmove(buffer_.data(),buffer_.data()+1,size_);}
  std::array<uint8_t,29> buffer_{};
  size_t size_=0;
};
} // namespace rm_chassis_comm

// [USB_SP 新增 开始] 伪终端模拟C板，不打开真实USB或驱动电机。
#include "io/gimbal/sp_link.hpp"
#include "tools/crc.hpp"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <limits>
#include <pty.h>
#include <stdexcept>
#include <unistd.h>
using namespace std::chrono_literals;
void require(bool value, const char * message) { if (!value) throw std::runtime_error(message); }
std::vector<uint8_t> status(uint8_t mode = 1) {
  io::GimbalToVision f{}; f.mode=mode; f.q[0]=1; f.bullet_speed=22;
  f.crc16=tools::get_crc16(reinterpret_cast<uint8_t *>(&f),sizeof(f)-2);
  const auto * b=reinterpret_cast<uint8_t *>(&f); return {b,b+sizeof(f)};
}
void send_bytes(int fd, const std::vector<uint8_t> & b) {
  require(::write(fd,b.data(),b.size())==static_cast<ssize_t>(b.size()),"emulator write");
}
std::vector<io::VisionToGimbal> collect(int fd,int duration_ms,int mode=-1) {
  std::vector<uint8_t> buf; std::vector<io::VisionToGimbal> out;
  auto end=io::SpClock::now()+std::chrono::milliseconds(duration_ms);
  while(io::SpClock::now()<end) {
    if(mode>=0) send_bytes(fd,status(mode));
    uint8_t bytes[2048]; auto n=::read(fd,bytes,sizeof(bytes));
    if(n>0)buf.insert(buf.end(),bytes,bytes+n);
    while(buf.size()>=29) {
      require(buf[0]=='S'&&buf[1]=='P',"TX header");
      require(tools::check_crc16(buf.data(),29),"TX CRC");
      io::VisionToGimbal f;std::memcpy(&f,buf.data(),29);out.push_back(f);
      buf.erase(buf.begin(),buf.begin()+29);
    }
    std::this_thread::sleep_for(5ms);
  }
  require(buf.empty(),"Unexpected partial TX");
  return out;
}
bool has_control(const std::vector<io::VisionToGimbal> & frames) {
  for(const auto & f:frames) { require(f.mode!=2,"fire must never be sent");if(f.mode==1)return true; }
  return false;
}
int main() try {
  require(tools::get_crc16(reinterpret_cast<const uint8_t *>("123456789"),9)==0x6f91,"CRC vector");
  int master,slave;char path[256]; require(openpty(&master,&slave,path,nullptr,nullptr)==0,"openpty");
  fcntl(master,F_SETFL,O_NONBLOCK);
  const std::string alias="/tmp/sp_link_test_"+std::to_string(getpid());
  std::filesystem::create_symlink(path,alias);
  {
    io::SpLink link(alias,60,120);
    for(int i=0;i<100&&!link.stats().connected;i++)std::this_thread::sleep_for(5ms);
    require(link.stats().connected,"open link");
    send_bytes(master,{0xa5,0x0e,0,0xae,2,0,1,30});
    require(collect(master,30).empty(),"must not transmit before valid SP");
    auto bad=status();bad[8]^=1;send_bytes(master,bad);
    auto invalid=status();float nan=std::numeric_limits<float>::quiet_NaN();
    std::memcpy(invalid.data()+3,&nan,4);auto crc=tools::get_crc16(invalid.data(),41);
    invalid[41]=crc&255;invalid[42]=crc>>8;send_bytes(master,invalid);
    require(collect(master,30).empty(),"bad CRC/NaN must not enable TX");
    require(link.stats().crc_errors>0&&link.stats().invalid_frames>0,"reject counters");
    for(size_t split=0;split<=43;split++) {
      auto f=status();send_bytes(master,std::vector<uint8_t>(f.begin(),f.begin()+split));
      std::this_thread::sleep_for(1ms);
      send_bytes(master,std::vector<uint8_t>(f.begin()+split,f.end()));
    }
    auto idle=collect(master,30,1); require(!idle.empty()&&!has_control(idle),"idle before target");
    require(link.stats().rx_frames>=44,"all fragmentation boundaries");
    auto feedback=link.feedback();require(feedback.has_value(),"real feedback available");
    require(link.feedback_at(io::SpClock::now()).has_value(),"image pose lookup");
    require(!link.feedback_at(io::SpClock::now()-1s),"old image has no matching pose");
    io::VisionToGimbal c{};c.mode=2;c.yaw=.25;c.pitch=-.10;
    c.yaw_vel=.2;c.yaw_acc=.3;c.pitch_vel=.4;c.pitch_acc=.5;
    link.submit(c,io::SpClock::now(),feedback->epoch);
    auto commands=collect(master,35,1);require(has_control(commands),"mode1 target send");
    for(const auto & f:commands) if(f.mode) require(
      f.yaw==c.yaw&&f.pitch==c.pitch&&f.yaw_vel==c.yaw_vel&&f.yaw_acc==c.yaw_acc&&
      f.pitch_vel==c.pitch_vel&&f.pitch_acc==c.pitch_acc,"29-byte field order");
    auto expired=collect(master,160,1);require(!expired.empty()&&expired.back().mode==0,"target expiration");
    feedback=link.feedback();link.submit(c,io::SpClock::now(),feedback->epoch);
    require(has_control(collect(master,25,1)),"fresh target resumes");
    auto lost=collect(master,85);require(!lost.empty()&&lost.back().mode==0&&!link.feedback(),"RX timeout stops control");
    require(!has_control(collect(master,35,1)),"RX recovery must not replay target");
    feedback=link.feedback();link.submit(c,io::SpClock::now(),feedback->epoch);
    require(has_control(collect(master,25,1)),"new target after RX recovery");
    auto manual=collect(master,30,0);require(!manual.empty()&&manual.back().mode==0,"remote mode overrides vision");
    require(!has_control(collect(master,30,1)),"switch back requires new target");
    feedback=link.feedback();c.yaw=10;link.submit(c,io::SpClock::now(),feedback->epoch);
    require(!has_control(collect(master,25,1)),"invalid target rejected");
    // PTY拔插模拟：实际工作线程必须清除会话和旧目标。
    c.yaw=.25;feedback=link.feedback();link.submit(c,io::SpClock::now(),feedback->epoch);
    require(has_control(collect(master,25,1)),"target before unplug");
    ::close(master);
    for(int i=0;i<100&&link.stats().connected;i++)std::this_thread::sleep_for(5ms);
    require(!link.stats().connected&&!link.feedback(),"disconnect clears feedback");
    ::close(slave);
    require(openpty(&master,&slave,path,nullptr,nullptr)==0,"replacement pty");
    fcntl(master,F_SETFL,O_NONBLOCK);
    std::filesystem::remove(alias);std::filesystem::create_symlink(path,alias);
    for(int i=0;i<300&&!link.stats().connected;i++)std::this_thread::sleep_for(5ms);
    require(link.stats().connected&&link.stats().reconnects>=2,"automatic reconnect");
    require(collect(master,20).empty(),"reconnect waits for SP");
    require(!has_control(collect(master,30,1)),"reconnect never replays old target");
  }
  auto stop=collect(master,25);require(!stop.empty()&&stop.back().mode==0,"exit idle frame");
  {
    io::SpLink receiver(path,60,120,false);
    for(int i=0;i<100&&!receiver.stats().connected;i++)std::this_thread::sleep_for(5ms);
    require(collect(master,40,1).empty(),"receive-only sends nothing");
    require(receiver.stats().rx_frames>0,"receive-only parses");
  }
  require(collect(master,20).empty(),"receive-only exit sends nothing");
  ::close(master);::close(slave);
  std::filesystem::remove(alias);
  std::cout<<"PASS: CRC, 44 splits, junk/NaN, field order, no fire, RX/target timeout, remote takeover, reconnect, pose lookup, exit, receive-only\n";
  return 0;
} catch(const std::exception & e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
// [USB_SP 新增 结束]

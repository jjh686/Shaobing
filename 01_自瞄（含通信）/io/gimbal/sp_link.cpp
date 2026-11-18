// [USB_SP 新增 开始] Linux USB CDC，单线程拥有fd，接收拆包/重连和100Hz发送。
#include "sp_link.hpp"
#include "tools/crc.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace io {
static_assert(sizeof(GimbalToVision) == 43);
static_assert(sizeof(VisionToGimbal) == 29);
static_assert(sizeof(float) == 4);
namespace {
double age_ms(SpClock::time_point a, SpClock::time_point b) {
  return std::chrono::duration<double, std::milli>(a - b).count();
}
bool valid_command(const VisionToGimbal & c) {
  return c.mode <= 2 && std::isfinite(c.yaw) && std::isfinite(c.pitch) &&
    std::isfinite(c.yaw_vel) && std::isfinite(c.pitch_vel) &&
    std::isfinite(c.yaw_acc) && std::isfinite(c.pitch_acc) &&
    std::abs(c.yaw) <= 3.141593f && std::abs(c.pitch) <= 1.570797f;
}
int open_port(const std::string & path) {
  int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) return -1;
  termios options{};
  if (flock(fd, LOCK_EX | LOCK_NB) || tcgetattr(fd, &options)) {
    int e = errno; ::close(fd); errno = e; return -1;
  }
  cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cc[VMIN] = 0; options.c_cc[VTIME] = 0;
  cfsetispeed(&options, B115200); cfsetospeed(&options, B115200);
  if (tcsetattr(fd, TCSANOW, &options)) {
    int e = errno; ::close(fd); errno = e; return -1;
  }
  // 独占防止另一个串口程序抢读；PTY测试也使用同一条路径。
  if (ioctl(fd, TIOCEXCL)) {
    int e = errno; ::close(fd); errno = e; return -1;
  }
  tcflush(fd, TCIFLUSH);
  return fd;
}
bool write_frame(int fd, const VisionToGimbal & frame) {
  const auto * bytes = reinterpret_cast<const uint8_t *>(&frame);
  size_t sent = 0;
  const auto deadline = SpClock::now() + std::chrono::milliseconds(20);
  while (sent < sizeof(frame) && SpClock::now() < deadline) {
    const auto n = ::write(fd, bytes + sent, sizeof(frame) - sent);
    if (n > 0) sent += static_cast<size_t>(n);
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
    else { pollfd p{fd, POLLOUT, 0}; ::poll(&p, 1, 2); }
  }
  return sent == sizeof(frame);
}
}
SpLink::SpLink(std::string port, int feedback_timeout_ms, int target_timeout_ms, bool transmit)
: port_(std::move(port)), feedback_timeout_ms_(feedback_timeout_ms),
  target_timeout_ms_(target_timeout_ms), transmit_(transmit) {
  const uint16_t endian = 1;
  if (*reinterpret_cast<const uint8_t *>(&endian) != 1)
    throw std::runtime_error("SP requires a little-endian host");
  if (port_.empty() || feedback_timeout_ms < 20 || feedback_timeout_ms > 500 ||
      target_timeout_ms < 20 || target_timeout_ms > 500)
    throw std::invalid_argument("SP port/timeouts invalid (20..500 ms)");
  worker_ = std::thread(&SpLink::run, this);
}
SpLink::~SpLink() { quit_ = true; if (worker_.joinable()) worker_.join(); }
bool SpLink::fresh_locked(SpClock::time_point now) const {
  return latest_ && stats_.connected && age_ms(now, latest_->received) <= feedback_timeout_ms_;
}
std::optional<SpFeedback> SpLink::feedback() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fresh_locked(SpClock::now()) ? latest_ : std::nullopt;
}
std::optional<SpFeedback> SpLink::feedback_at(SpClock::time_point t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!fresh_locked(SpClock::now()) || history_.empty()) return std::nullopt;
  auto b = std::lower_bound(history_.begin(), history_.end(), t,
    [](const SpFeedback & f, SpClock::time_point value) { return f.received < value; });
  if (b == history_.begin() || b == history_.end()) {
    const auto & nearest = b == history_.end() ? history_.back() : *b;
    if (std::abs(age_ms(nearest.received, t)) > 50) return std::nullopt;
    return nearest;
  }
  const auto & a = *std::prev(b);
  if (age_ms(b->received, a.received) > 50 || a.epoch != b->epoch) return std::nullopt;
  auto result = a;
  const double span = age_ms(b->received, a.received);
  result.q = a.q.slerp(span > 0 ? age_ms(t, a.received) / span : 0, b->q).normalized();
  return result;
}
SpStats SpLink::stats() const { std::lock_guard<std::mutex> lock(mutex_); return stats_; }
void SpLink::clear_command() {
  std::lock_guard<std::mutex> lock(mutex_); command_ = {}; image_time_ = {};
}
void SpLink::submit(VisionToGimbal c, SpClock::time_point t, uint64_t epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  // 本次只做云台联动。即使Planner要求开火，也只发mode1。
  if (!valid_command(c)) c = {};
  if (c.mode) c.mode = 1;
  command_ = c; image_time_ = t; command_epoch_ = epoch;
}
VisionToGimbal SpLink::outgoing() {
  std::lock_guard<std::mutex> lock(mutex_);
  VisionToGimbal out{};
  const auto now = SpClock::now();
  if (fresh_locked(now) && latest_->mode == 1 && command_epoch_ == latest_->epoch &&
      age_ms(now, image_time_) >= 0 && age_ms(now, image_time_) <= target_timeout_ms_ &&
      valid_command(command_) && command_.mode) out = command_;
  out.head[0] = 'S'; out.head[1] = 'P';
  out.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t *>(&out), sizeof(out) - 2);
  return out;
}
void SpLink::ingest(std::vector<uint8_t> & buffer) {
  while (!buffer.empty()) {
    if (buffer[0] != 'S' || (buffer.size() > 1 && buffer[1] != 'P')) {
      buffer.erase(buffer.begin()); continue;
    }
    if (buffer.size() < sizeof(GimbalToVision)) return;
    GimbalToVision frame; std::memcpy(&frame, buffer.data(), sizeof(frame));
    std::lock_guard<std::mutex> lock(mutex_);
    if (!tools::check_crc16(buffer.data(), sizeof(frame))) {
      ++stats_.crc_errors; buffer.erase(buffer.begin()); continue;
    }
    Eigen::Quaterniond q(frame.q[0], frame.q[1], frame.q[2], frame.q[3]);
    bool valid = frame.mode <= 3 && q.coeffs().allFinite() && q.squaredNorm() > 0.81 &&
      q.squaredNorm() < 1.21 && std::isfinite(frame.yaw) && std::isfinite(frame.pitch) &&
      std::isfinite(frame.yaw_vel) && std::isfinite(frame.pitch_vel) &&
      std::isfinite(frame.bullet_speed) && frame.bullet_speed >= 0 &&
      std::abs(frame.yaw) <= 3.141593f && std::abs(frame.pitch) <= 1.570797f;
    if (!valid) { ++stats_.invalid_frames; buffer.erase(buffer.begin()); continue; }
    auto now = SpClock::now();
    if (!fresh_locked(now) || latest_->mode != frame.mode) {
      ++epoch_; command_ = {}; history_.clear();
    }
    SpFeedback f; f.q = q.normalized(); f.mode = frame.mode; f.epoch = epoch_; f.received = now;
    f.state = {frame.yaw, frame.yaw_vel, frame.pitch, frame.pitch_vel,
               frame.bullet_speed, frame.bullet_count};
    latest_ = f; history_.push_back(f);
    while (history_.size() > 1 && age_ms(now, history_.front().received) > 250) history_.pop_front();
    ++stats_.rx_frames;
    buffer.erase(buffer.begin(), buffer.begin() + sizeof(frame));
  }
}
void SpLink::run() {
  int fd = -1; bool session_sp = false;
  std::vector<uint8_t> buffer;
  auto next_open = SpClock::now(), next_tx = next_open;
  auto disconnect = [&] {
    if (fd >= 0) { ioctl(fd, TIOCNXCL); ::close(fd); fd = -1; }
    buffer.clear(); session_sp = false; next_open = SpClock::now() + std::chrono::seconds(1);
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.connected = false; latest_.reset(); history_.clear(); command_ = {}; ++epoch_;
  };
  while (!quit_) {
    if (fd < 0) {
      if (SpClock::now() < next_open) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
      fd = open_port(port_);
      if (fd < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.error = std::strerror(errno); next_open = SpClock::now() + std::chrono::seconds(1);
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.connected = true; stats_.error.clear(); ++stats_.reconnects; next_tx = SpClock::now();
    }
    pollfd p{fd, POLLIN, 0};
    const int result = ::poll(&p, 1, 5);
    if ((result < 0 && errno != EINTR) || (p.revents & (POLLHUP | POLLERR | POLLNVAL))) {
      { std::lock_guard<std::mutex> lock(mutex_); stats_.error = "USB disconnected/read error"; }
      disconnect(); continue;
    }
    if (p.revents & POLLIN) {
      uint8_t bytes[1024]; const auto n = ::read(fd, bytes, sizeof(bytes));
      if (n > 0) {
        { std::lock_guard<std::mutex> lock(mutex_); stats_.rx_bytes += n; }
        buffer.insert(buffer.end(), bytes, bytes + n); ingest(buffer);
        if (feedback()) session_sp = true;
      } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        disconnect(); continue;
      }
    }
    // 启动必须先收到有效SP状态；旧A5协议或噪声不触发任何发送。
    if (transmit_ && session_sp && SpClock::now() >= next_tx) {
      const auto frame = outgoing();
      if (!write_frame(fd, frame)) {
        { std::lock_guard<std::mutex> lock(mutex_); stats_.error = "USB write failed/short write"; }
        disconnect(); continue;
      }
      { std::lock_guard<std::mutex> lock(mutex_); ++stats_.tx_frames;
        stats_.last_tx_mode = frame.mode; if (frame.mode) ++stats_.tx_control_frames; }
      next_tx = SpClock::now() + std::chrono::milliseconds(10);
    }
  }
  if (fd >= 0 && transmit_ && session_sp) {
    clear_command(); const auto stop = outgoing(); write_frame(fd, stop);
  }
  disconnect();
}
}  // namespace io
// [USB_SP 新增 结束]

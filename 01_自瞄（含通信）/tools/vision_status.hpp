#ifndef TOOLS__VISION_STATUS_HPP
#define TOOLS__VISION_STATUS_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <string_view>

namespace tools
{
class VisionStatus
{
public:
  explicit VisionStatus(bool enabled) noexcept
  {
    if (!enabled) return;
    socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    decision_.sin_family = AF_INET;
    decision_.sin_port = ::htons(9871);
    decision_.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  }

  ~VisionStatus()
  {
    if (socket_ >= 0) ::close(socket_);
  }

  VisionStatus(const VisionStatus &) = delete;
  VisionStatus & operator=(const VisionStatus &) = delete;

  void publish(std::chrono::steady_clock::time_point frame_stamp, std::string_view state) noexcept
  {
    if (socket_ < 0) return;
    const auto now = std::chrono::steady_clock::now();
    if (now < next_publish_ || frame_stamp > now || frame_stamp <= last_frame_) return;
    next_publish_ = now + std::chrono::milliseconds(100);
    last_frame_ = frame_stamp;

    if (state != "lost" && state != "detecting" && state != "tracking" && state != "temp_lost" &&
        state != "switching") {
      state = "unavailable";
    }
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
      frame_stamp.time_since_epoch()).count();
    char frame[64];
    const auto size = std::snprintf(
      frame, sizeof(frame), "RMVISION1 %" PRId64 " %.*s\n", static_cast<std::int64_t>(nanoseconds),
      static_cast<int>(state.size()), state.data());
    if (size <= 0 || static_cast<std::size_t>(size) >= sizeof(frame)) return;
    ::sendto(socket_, frame, size, 0, reinterpret_cast<const sockaddr *>(&decision_), sizeof(decision_));
  }

private:
  int socket_{-1};
  sockaddr_in decision_{};
  std::chrono::steady_clock::time_point next_publish_{};
  std::chrono::steady_clock::time_point last_frame_{};
};
}  // namespace tools

#endif  // TOOLS__VISION_STATUS_HPP

// [USB_SP 新增 开始] 桌面实时视觉专用连接；复用已有29/43字节SP结构。
#pragma once
#include "gimbal.hpp"
#include <deque>
#include <optional>
#include <vector>

namespace io {
using SpClock = std::chrono::steady_clock;
struct SpFeedback {
  Eigen::Quaterniond q{1, 0, 0, 0};  // 板端原始wxyz，安装矩阵由Solver统一应用
  GimbalState state{};
  uint8_t mode = 0;
  uint64_t epoch = 0;
  SpClock::time_point received{};
};
struct SpStats {
  bool connected = false;
  uint64_t rx_bytes = 0, rx_frames = 0, crc_errors = 0, invalid_frames = 0;
  uint64_t tx_frames = 0, tx_control_frames = 0, reconnects = 0;
  uint8_t last_tx_mode = 0;
  std::string error;
};
class SpLink {
 public:
  SpLink(std::string port, int feedback_timeout_ms = 100, int target_timeout_ms = 100,
         bool transmit = true);
  ~SpLink();
  SpLink(const SpLink &) = delete;
  SpLink & operator=(const SpLink &) = delete;
  std::optional<SpFeedback> feedback() const;
  std::optional<SpFeedback> feedback_at(SpClock::time_point image_time) const;
  SpStats stats() const;
  // 目标保质期从图像采集时刻算；mode/会话变化后旧目标不得重放。
  void submit(VisionToGimbal command, SpClock::time_point image_time, uint64_t epoch);
  void clear_command();
 private:
  std::string port_;
  int feedback_timeout_ms_, target_timeout_ms_;
  bool transmit_;
  std::atomic<bool> quit_{false};
  std::thread worker_;
  mutable std::mutex mutex_;
  SpStats stats_;
  std::optional<SpFeedback> latest_;
  std::deque<SpFeedback> history_;
  uint64_t epoch_ = 0, command_epoch_ = 0;
  VisionToGimbal command_{};
  SpClock::time_point image_time_{};
  void run();
  void ingest(std::vector<uint8_t> & buffer);
  bool fresh_locked(SpClock::time_point now) const;
  VisionToGimbal outgoing();
};
}  // namespace io
// [USB_SP 新增 结束]

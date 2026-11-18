#ifndef RM_CHASSIS_COMM__GIMBAL_CLOCK_HPP_
#define RM_CHASSIS_COMM__GIMBAL_CLOCK_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace rm_chassis_comm
{

// Maps MCU millisecond ticks to the caller's host steady-clock epoch. The caller
// must authenticate the echoed command/session and reset on a session change.
// This is an NTP-style estimate, not hardware clock synchronization.
class GimbalClock
{
public:
  struct Result
  {
    double sample_host_ms;
    double uncertainty_ms;
  };

  struct Limits
  {
    double max_processing_ms = 200.0;
    double max_sample_age_ms = 50.0;
    double max_host_rtt_ms = 100.0;
    double max_network_rtt_ms = 20.0;
    double max_mapped_age_ms = 100.0;
    double future_tolerance_ms = 1.0;
  };

  GimbalClock() = default;
  explicit GimbalClock(const Limits & limits) : limits_(limits) {}

  void reset()
  {
    initialized_ = false;
    count_ = 0;
    next_ = 0;
    last_uptime_ = 0;
    last_sample_ = 0;
    unwrapped_uptime_ = 0.0;
    last_sample_host_ = 0.0;
  }

  std::optional<Result> observe(
    uint32_t sender_uptime, uint32_t command_received, uint32_t sample_tick,
    double host_send_ms, double host_receive_ms)
  {
    if (!std::isfinite(host_send_ms) || !std::isfinite(host_receive_ms)) {
      return std::nullopt;
    }
    const double host_rtt = host_receive_ms - host_send_ms;
    if (host_rtt < 0.0 || host_rtt > limits_.max_host_rtt_ms) {
      return std::nullopt;
    }

    // Unsigned subtraction handles the uint32 wrap. Any backwards timestamp
    // produces a large delta and is rejected by the bounded interval below.
    const double processing_ms = static_cast<uint32_t>(sender_uptime - command_received);
    const double sample_age_ms = static_cast<uint32_t>(sender_uptime - sample_tick);
    if (processing_ms > limits_.max_processing_ms ||
      sample_age_ms > limits_.max_sample_age_ms)
    {
      return std::nullopt;
    }
    const double network_rtt = host_rtt - processing_ms;
    if (network_rtt < 0.0 || network_rtt > limits_.max_network_rtt_ms) {
      return std::nullopt;
    }

    double tx_ms = static_cast<double>(sender_uptime);
    if (initialized_) {
      const uint32_t tx_delta = sender_uptime - last_uptime_;
      const uint32_t sample_delta = sample_tick - last_sample_;
      if (!forward(tx_delta) || !forward(sample_delta)) {
        return std::nullopt;
      }
      tx_ms = unwrapped_uptime_ + static_cast<double>(tx_delta);
    }
    const double rx_ms = tx_ms - processing_ms;
    const Entry candidate{
      ((host_send_ms - rx_ms) + (host_receive_ms - tx_ms)) / 2.0,
      network_rtt};

    // Evaluate the prospective window without mutating it: a rejected frame
    // must not evict a good estimate or advance any accepted-time baseline.
    Entry best = candidate;
    for (std::size_t i = 0; i < count_; ++i) {
      if (count_ == window_.size() && i == next_) {
        continue;
      }
      if (window_[i].network_rtt_ms < best.network_rtt_ms) {
        best = window_[i];
      }
    }
    const double mapped_sample = tx_ms - sample_age_ms + best.host_minus_mcu_ms;
    const double mapped_age = host_receive_ms - mapped_sample;
    if (!std::isfinite(mapped_sample) ||
      mapped_age < -limits_.future_tolerance_ms ||
      mapped_age > limits_.max_mapped_age_ms ||
      (initialized_ && mapped_sample <= last_sample_host_))
    {
      return std::nullopt;
    }

    window_[next_] = candidate;
    next_ = (next_ + 1) % window_.size();
    if (count_ < window_.size()) {
      ++count_;
    }
    initialized_ = true;
    last_uptime_ = sender_uptime;
    last_sample_ = sample_tick;
    unwrapped_uptime_ = tx_ms;
    last_sample_host_ = mapped_sample;
    return Result{mapped_sample, best.network_rtt_ms / 2.0};
  }

private:
  struct Entry
  {
    double host_minus_mcu_ms = 0.0;
    double network_rtt_ms = 0.0;
  };

  static bool forward(uint32_t delta)
  {
    // Half-range ordering is unambiguous provided accepted observations are
    // less than 2^31 ms apart (about 24.8 days).
    return delta != 0 && delta < UINT32_C(0x80000000);
  }

  Limits limits_{};
  std::array<Entry, 16> window_{};
  std::size_t count_ = 0;
  std::size_t next_ = 0;
  bool initialized_ = false;
  uint32_t last_uptime_ = 0;
  uint32_t last_sample_ = 0;
  double unwrapped_uptime_ = 0.0;
  double last_sample_host_ = 0.0;
};

}  // namespace rm_chassis_comm

#endif  // RM_CHASSIS_COMM__GIMBAL_CLOCK_HPP_

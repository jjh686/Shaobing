#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>
#include <unistd.h>

#include <fmt/chrono.h>
#include <nlohmann/json.hpp>

#include "logger.hpp"

namespace tools
{
// Host timestamps measure SDK delivery -> completed processing; they do not include
// exposure or camera-to-host transport. USB TX counts writes, not board acknowledgements.
class RuntimeStats
{
public:
  using Clock = std::chrono::steady_clock;
  // Camera deliveries, SDK sequence gaps, USB RX, USB TX, CRC errors, invalid frames.
  using Counters = std::array<uint64_t, 6>;
  struct Sample
  {
    // Wait for image, detection, tracking, aim/plan/panel, GUI, whole loop, frame age.
    std::array<double, 7> ms;
    bool detected, tracked, pose_valid;
  };

  RuntimeStats(Counters initial, bool live_camera)
  : initial_(initial), previous_(initial), live_camera_(live_camera)
  {
    std::filesystem::create_directories("logs");
    path_ = fmt::format("logs/performance_{:%Y%m%d_%H%M%S}_{}.jsonl",
                        std::chrono::system_clock::now(), ::getpid());
    file_.open(path_);
    samples_.reserve(256);
    logger()->info("[PERF] Online performance log: {}", path_);
  }

  bool add(Sample sample, Clock::time_point now)
  {
    samples_.push_back(sample);
    return now - window_start_ >= std::chrono::seconds(1);
  }

  void publish(Counters counters, const std::string & color, Clock::time_point now)
  {
    if (samples_.empty()) return;
    const double seconds = std::chrono::duration<double>(now - window_start_).count();
    const double fps = samples_.size() / seconds;
    const double camera_fps = (counters[0] - previous_[0]) / seconds;
    const double rx_hz = (counters[2] - previous_[2]) / seconds;
    const double tx_hz = (counters[3] - previous_[3]) / seconds;
    static constexpr const char * names[] = {
      "camera_wait_ms", "detect_ms", "track_ms", "aim_plan_panel_ms", "gui_ms", "loop_ms",
      "sdk_to_finish_ms"};
    nlohmann::json timings;
    for (std::size_t column = 0; column < 7; ++column) {
      std::vector<double> values;
      values.reserve(samples_.size());
      for (const auto & s : samples_) values.push_back(s.ms[column]);
      std::sort(values.begin(), values.end());
      auto quantile = [&](double fraction) {
        return values[static_cast<std::size_t>(std::ceil(fraction * values.size())) - 1];
      };
      timings[names[column]] = {
        {"mean", std::accumulate(values.begin(), values.end(), 0.0) / values.size()},
        {"p50", quantile(0.5)}, {"p95", quantile(0.95)}, {"max", values.back()}};
    }
    std::size_t detected = 0, tracked = 0, valid_pose = 0;
    for (const auto & s : samples_) {
      detected += s.detected;
      tracked += s.tracked;
      valid_pose += s.pose_valid;
    }
    const double age = timings["sdk_to_finish_ms"]["p95"].get<double>();
    overlay_ = fmt::format("FPS {:.1f} | CAM {:.1f} | RX {:.0f} TX {:.0f} Hz | age95 {:.1f}ms",
                            fps, camera_fps, rx_hz, tx_hz, age);
    nlohmann::json row = {
      {"time", fmt::format("{:%Y-%m-%d %H:%M:%S}", std::chrono::system_clock::now())},
      {"elapsed_s", std::chrono::duration<double>(now - start_).count()},
      {"window_s", seconds}, {"frames", samples_.size()}, {"vision_fps", fps},
      {"camera_fps", camera_fps}, {"rx_hz", rx_hz}, {"tx_hz", tx_hz},
      {"sdk_frame_gaps", counters[1] - previous_[1]},
      {"crc_errors", counters[4] - previous_[4]},
      {"invalid_frames", counters[5] - previous_[5]},
      {"camera_frames_total", counters[0] - initial_[0]},
      {"rx_total", counters[2] - initial_[2]}, {"tx_total", counters[3] - initial_[3]},
      {"detected_frames", detected}, {"tracked_frames", tracked},
      {"valid_pose_frames", valid_pose}, {"enemy_color", color},
      {"live_camera", live_camera_}, {"timings", timings}};
    file_ << row.dump() << '\n';
    file_.flush();
    logger()->info("[PERF] {} | detect {:.1f}ms GUI {:.1f}ms | tracking {}/{}",
                   overlay_, timings["detect_ms"]["mean"].get<double>(),
                   timings["gui_ms"]["mean"].get<double>(), tracked, samples_.size());
    previous_ = counters;
    samples_.clear();
    window_start_ = now;
  }

  const std::string & overlay() const { return overlay_; }

private:
  Counters initial_, previous_;
  bool live_camera_;
  Clock::time_point start_ = Clock::now(), window_start_ = start_;
  std::string path_, overlay_ = "FPS: collecting online statistics...";
  std::ofstream file_;
  std::vector<Sample> samples_;
};
}  // namespace tools

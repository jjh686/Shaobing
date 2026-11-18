#ifndef RM_CHASSIS_COMM_FRAME_MATH_HPP
#define RM_CHASSIS_COMM_FRAME_MATH_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace rm_chassis_comm::frame_math {

constexpr double kPi = 3.141592653589793238462643383279502884;

inline double wrap_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

struct PlanarTwist {
  double vx{}, vy{}, vz{}, wx{}, wy{}, wz{};
};

inline bool finite_planar(const PlanarTwist & v, double tolerance = 1e-9)
{
  return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.vz) &&
         std::isfinite(v.wx) && std::isfinite(v.wy) && std::isfinite(v.wz) &&
         std::fabs(v.vz) <= tolerance && std::fabs(v.wx) <= tolerance &&
         std::fabs(v.wy) <= tolerance;
}

// The source twist is at the yaw joint origin. pivot is that origin in the
// body frame. v_body = Rz(yaw) * v_yaw + pivot cross omega. Angular velocity
// is already the requested chassis spin and is preserved exactly once.
inline std::optional<PlanarTwist> to_body(
  const PlanarTwist & source, double yaw, const std::array<double, 3> & pivot)
{
  if (!finite_planar(source) || !std::isfinite(yaw)) {
    return std::nullopt;
  }
  for (double p : pivot) {
    if (!std::isfinite(p)) {
      return std::nullopt;
    }
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  PlanarTwist result;
  result.vx = c * source.vx - s * source.vy + pivot[1] * source.wz;
  result.vy = s * source.vx + c * source.vy - pivot[0] * source.wz;
  result.wz = source.wz;
  if (!finite_planar(result)) {
    return std::nullopt;
  }
  return result;
}

struct JointSample {
  std::int64_t stamp_ns{};
  double yaw{}, pitch{};
};

// Strictly increasing timestamps, bounded storage, and interpolation only.
// An exact endpoint is valid; timestamps outside the stored interval fail.
class JointHistory {
public:
  static constexpr std::size_t kCapacity = 64;

  void clear() {size_ = 0;}
  std::size_t size() const {return size_;}
  const JointSample * first() const {return size_ ? &samples_[0] : nullptr;}
  const JointSample * last() const {return size_ ? &samples_[size_ - 1] : nullptr;}

  bool push(JointSample sample)
  {
    if (sample.stamp_ns <= 0 || !std::isfinite(sample.yaw) ||
      !std::isfinite(sample.pitch) ||
      (size_ && sample.stamp_ns <= samples_[size_ - 1].stamp_ns))
    {
      return false;
    }
    sample.yaw = wrap_angle(sample.yaw);
    if (size_ == kCapacity) {
      for (std::size_t i = 1; i < size_; ++i) {
        samples_[i - 1] = samples_[i];
      }
      --size_;
    }
    samples_[size_++] = sample;
    return true;
  }

  std::optional<JointSample> at(std::int64_t stamp_ns) const
  {
    if (!size_ || stamp_ns < samples_[0].stamp_ns ||
      stamp_ns > samples_[size_ - 1].stamp_ns)
    {
      return std::nullopt;
    }
    for (std::size_t i = 0; i < size_; ++i) {
      if (stamp_ns == samples_[i].stamp_ns) {
        return samples_[i];
      }
      if (i && stamp_ns < samples_[i].stamp_ns) {
        const auto & before = samples_[i - 1];
        const auto & after = samples_[i];
        const double t = static_cast<double>(stamp_ns - before.stamp_ns) /
          static_cast<double>(after.stamp_ns - before.stamp_ns);
        const JointSample result{
          stamp_ns,
          wrap_angle(before.yaw + t * wrap_angle(after.yaw - before.yaw)),
          (1.0 - t) * before.pitch + t * after.pitch};
        if (!std::isfinite(result.yaw) || !std::isfinite(result.pitch)) {
          return std::nullopt;
        }
        return result;
      }
    }
    return std::nullopt;
  }

private:
  std::array<JointSample, kCapacity> samples_{};
  std::size_t size_{};
};

}  // namespace rm_chassis_comm::frame_math
#endif

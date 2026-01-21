#pragma once

#include <array>
#include <cstdint>

namespace aimrl_sdk {

constexpr int kArmDof = 14;
constexpr int kLegDof = 12;
constexpr int kFrameDim = 88;

struct TimestampNs {
  std::int64_t value{0};
  friend constexpr auto operator<=>(TimestampNs, TimestampNs) = default;
};

struct Sequence32 {
  std::uint32_t value{0};
  friend constexpr auto operator<=>(Sequence32, Sequence32) = default;
};

enum class Field : std::uint32_t {
  Position = 1u << 0,
  Velocity = 1u << 1,
  Effort = 1u << 2,
  Stiffness = 1u << 3,
  Damping = 1u << 4,
};

constexpr std::uint32_t to_mask(Field f) noexcept {
  return static_cast<std::uint32_t>(f);
}

template <int DOF> struct JointSample {
  TimestampNs stamp{};
  Sequence32 header_seq{};
  std::array<std::uint32_t, DOF> joint_seq{};
  std::array<double, DOF> pos{};
  std::array<double, DOF> vel{};
  std::array<double, DOF> eff{};
};

struct ImuSample {
  TimestampNs stamp{};
  Sequence32 header_seq{};
  std::array<double, 4> quat_xyzw{0, 0, 0, 1};
  std::array<double, 3> gyro{};
  std::array<double, 3> acc{};
};

struct Frame {
  TimestampNs stamp{};
  // `complete`: whether arm+leg+imu samples were all available for this tick.
  // `aligned`: whether the complete frame passes the time-skew check.
  bool aligned{false};
  bool complete{false};
  std::int64_t skew_ns{0};
  std::array<float, kFrameDim> x{};
};

template <int DOF> struct PendingCommand {
  bool has_any{false};
  std::uint32_t mask{0};

  std::array<double, DOF> pos{};
  std::array<double, DOF> vel{};
  std::array<double, DOF> eff{};
  std::array<double, DOF> kp{};
  std::array<double, DOF> kd{};
};

struct SyncConfig {
  double frame_hz{100.0};
  std::int64_t max_skew_ns{3'000'000}; // 3ms
  int max_backtrack{200};
};

} // namespace aimrl_sdk

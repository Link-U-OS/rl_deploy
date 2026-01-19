#include "core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include "layout.hpp"

namespace aimrl_sdk {

static TimestampNs now_system_ns() {
  using namespace std::chrono;
  const auto ns =
      duration_cast<nanoseconds>(system_clock::now().time_since_epoch())
          .count();
  return TimestampNs{ns};
}

Core::Core(Options opt, std::unique_ptr<Transport> transport)
    : opt_(std::move(opt)), transport_(std::move(transport)), arm_raw_(opt_.raw_ring), leg_raw_(opt_.raw_ring), imu_raw_(opt_.raw_ring), frame_ring_(opt_.frame_ring) {
  if (!transport_)
    throw std::invalid_argument("transport is null");

  if (opt_.arm_names.size() != kArmDof)
    throw std::invalid_argument("arm_names size != 14");
  if (opt_.leg_names.size() != kLegDof)
    throw std::invalid_argument("leg_names size != 12");

  const auto frame_hz = opt_.sync.frame_hz;
  if (!std::isfinite(frame_hz) || frame_hz <= 0.0)
    throw std::invalid_argument("sync.frame_hz must be > 0");
  const auto period_ns_f = 1e9 / frame_hz;
  const auto max_period_ns =
      static_cast<double>(std::numeric_limits<std::int64_t>::max());
  if (!std::isfinite(period_ns_f) || period_ns_f < 1.0 ||
      period_ns_f > max_period_ns) {
    throw std::invalid_argument(
        "sync.frame_hz out of range (computed period_ns invalid)");
  }
}

Core::~Core() { stop(); }

void Core::start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true))
    return;

  Transport::Callbacks callbacks{
      .on_arm_state = [this](const auto &msg) { on_arm_state(msg); },
      .on_leg_state = [this](const auto &msg) { on_leg_state(msg); },
      .on_imu = [this](const auto &msg) { on_imu(msg); },
  };
  transport_->start(std::move(callbacks));

  sync_thread_ = std::jthread([this](const std::stop_token &st) { sync_loop_(st); });
}

void Core::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false))
    return;

  if (sync_thread_.joinable()) {
    sync_thread_.request_stop();
    sync_thread_.join();
  }

  transport_->stop();
  frame_cv_.notify_all();
}

void Core::on_arm_state(const std::shared_ptr<const joint_msgs::msg::JointState> &msg) {
  JointSample<kArmDof> arm{};
  const auto arm_stamp =
      static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg->header.stamp.nanosec);
  arm.stamp = (arm_stamp > 0) ? TimestampNs{arm_stamp} : now_system_ns();
  for (int i = 0; i < kArmDof; ++i) {
    arm.pos[i] = msg->joints[i].position;
    arm.vel[i] = msg->joints[i].velocity;
    arm.eff[i] = msg->joints[i].effort;
    // arm.joint_seq[i] = static_cast<std::uint32_t>(msg->joints[i].name.size());
    arm.header_seq = Sequence32{static_cast<std::uint32_t>(msg->header.stamp.nanosec)};
  }
  arm_raw_.write([&](auto &dst) { dst = arm; });
}

void Core::on_leg_state(const std::shared_ptr<const joint_msgs::msg::JointState> &msg) {
  JointSample<kLegDof> leg{};
  const auto leg_stamp =
      static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg->header.stamp.nanosec);
  leg.stamp = (leg_stamp > 0) ? TimestampNs{leg_stamp} : now_system_ns();
  for (int i = 0; i < kLegDof; ++i) {
    leg.pos[i] = msg->joints[i].position;
    leg.vel[i] = msg->joints[i].velocity;
    leg.eff[i] = msg->joints[i].effort;
    // leg.joint_seq[i] = static_cast<std::uint32_t>(msg->joints[i].name.size());
    leg.header_seq = Sequence32{static_cast<std::uint32_t>(msg->header.stamp.nanosec)};
  }
  leg_raw_.write([&](auto &dst) { dst = leg; });
}

void Core::on_imu(const std::shared_ptr<const sensor_msgs::msg::Imu> &msg) {
  ImuSample imu{};
  const auto imu_stamp =
      static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg->header.stamp.nanosec);
  imu.stamp = (imu_stamp > 0) ? TimestampNs{imu_stamp} : now_system_ns();
  imu.quat_xyzw[0] = msg->orientation.x;
  imu.quat_xyzw[1] = msg->orientation.y;
  imu.quat_xyzw[2] = msg->orientation.z;
  imu.quat_xyzw[3] = msg->orientation.w;
  imu.gyro[0] = msg->angular_velocity.x;
  imu.gyro[1] = msg->angular_velocity.y;
  imu.gyro[2] = msg->angular_velocity.z;
  imu.acc[0] = msg->linear_acceleration.x;
  imu.acc[1] = msg->linear_acceleration.y;
  imu.acc[2] = msg->linear_acceleration.z;
  imu_raw_.write([&](auto &dst) { dst = imu; });
}

std::optional<Frame> Core::latest_frame() const {
  const auto ridx = frame_ring_.latest_index();
  if (ridx == 0)
    return std::nullopt;
  Frame f{};
  if (!frame_ring_.read_at(ridx, f))
    return std::nullopt;
  return f;
}

std::optional<Frame> Core::wait_next_frame(std::uint64_t after_seq,
                                           std::optional<double> timeout_s) {
  std::unique_lock lk(frame_mtx_);
  auto pred = [&] {
    return frame_seq_.load(std::memory_order_relaxed) > after_seq || !running();
  };

  if (timeout_s.has_value()) {
    const auto dur = std::chrono::duration<double>(*timeout_s);
    if (!frame_cv_.wait_for(lk, dur, pred))
      return std::nullopt;
  } else {
    frame_cv_.wait(lk, pred);
  }

  if (!running())
    return std::nullopt;
  return latest_frame();
}

std::vector<Frame> Core::read_last_frames(int n) const {
  if (n <= 0)
    throw std::invalid_argument("n must be > 0");

  std::vector<Frame> out(static_cast<std::size_t>(n));
  const auto latest = frame_ring_.latest_index();
  int got = 0;
  for (int i = 0; i < n; ++i) {
    const auto idx = (latest > static_cast<std::uint64_t>(i))
                         ? (latest - static_cast<std::uint64_t>(i))
                         : 0;
    if (idx == 0)
      break;
    Frame f{};
    if (!frame_ring_.read_at(idx, f))
      break;
    out[static_cast<std::size_t>(n - 1 - i)] = f;  // reverse into oldest->newest
    ++got;
  }
  // pad remaining leading frames default-initialized (valid=false)
  (void)got;
  return out;
}

template <class Ring, class Sample>
bool Core::find_leq_(const Ring &ring, TimestampNs t, std::uint64_t start_idx,
                     int max_backtrack, Sample &out) {
  auto idx = start_idx;
  Sample tmp{};
  for (int i = 0; i < max_backtrack && idx > 0; ++i, --idx) {
    if (!ring.read_at(idx, tmp))
      continue;
    if (tmp.stamp.value > 0 && tmp.stamp.value <= t.value) {
      out = tmp;
      return true;
    }
  }
  return false;
}

static void fill_arm_leg_imu(Frame &out, const JointSample<kArmDof> &arm,
                             const JointSample<kLegDof> &leg,
                             const ImuSample &imu) {
  // arm
  for (int i = 0; i < kArmDof; ++i) {
    out.x[FrameLayout::ArmPos0 + i] =
        static_cast<float>(arm.pos[static_cast<std::size_t>(i)]);
    out.x[FrameLayout::ArmVel0 + i] =
        static_cast<float>(arm.vel[static_cast<std::size_t>(i)]);
    out.x[FrameLayout::ArmEff0 + i] =
        static_cast<float>(arm.eff[static_cast<std::size_t>(i)]);
  }
  // leg
  for (int i = 0; i < kLegDof; ++i) {
    out.x[FrameLayout::LegPos0 + i] =
        static_cast<float>(leg.pos[static_cast<std::size_t>(i)]);
    out.x[FrameLayout::LegVel0 + i] =
        static_cast<float>(leg.vel[static_cast<std::size_t>(i)]);
    out.x[FrameLayout::LegEff0 + i] =
        static_cast<float>(leg.eff[static_cast<std::size_t>(i)]);
  }
  // imu
  for (int i = 0; i < 4; ++i)
    out.x[FrameLayout::ImuQuat0 + i] =
        static_cast<float>(imu.quat_xyzw[static_cast<std::size_t>(i)]);
  for (int i = 0; i < 3; ++i)
    out.x[FrameLayout::ImuGyro0 + i] =
        static_cast<float>(imu.gyro[static_cast<std::size_t>(i)]);
  for (int i = 0; i < 3; ++i)
    out.x[FrameLayout::ImuAcc0 + i] =
        static_cast<float>(imu.acc[static_cast<std::size_t>(i)]);
}

void Core::sync_loop_(const std::stop_token &stoken) {
  const auto period_ns = static_cast<std::int64_t>(1e9 / opt_.sync.frame_hz);

  auto next = now_system_ns();
  next.value = (next.value / period_ns + 1) * period_ns;

  while (!stoken.stop_requested() && running()) {
    const auto tp = std::chrono::system_clock::time_point(
        std::chrono::nanoseconds(next.value));
    std::this_thread::sleep_until(tp);
    const auto tick = TimestampNs{next.value};
    next.value += period_ns;

    // fetch samples <= tick
    JointSample<kArmDof> arm{};
    JointSample<kLegDof> leg{};
    ImuSample imu{};

    const auto arm_ok = find_leq_(arm_raw_, tick, arm_raw_.latest_index(),
                                  opt_.sync.max_backtrack, arm);
    const auto leg_ok = find_leq_(leg_raw_, tick, leg_raw_.latest_index(),
                                  opt_.sync.max_backtrack, leg);
    const auto imu_ok = find_leq_(imu_raw_, tick, imu_raw_.latest_index(),
                                  opt_.sync.max_backtrack, imu);

    Frame out{};
    out.stamp = tick;

    if (opt_.sync.require_all && !(arm_ok && leg_ok && imu_ok)) {
      out.valid = false;
      if (!opt_.sync.drop_invalid) {
        frame_ring_.write([&](Frame &dst) { dst = out; });
        frame_seq_.fetch_add(1, std::memory_order_relaxed);
        frame_cv_.notify_all();
      }
      continue;
    }

    // skew w.r.t tick
    std::int64_t skew = 0;
    if (arm_ok)
      skew = std::max(skew, static_cast<std::int64_t>(
                                std::llabs(arm.stamp.value - tick.value)));
    if (leg_ok)
      skew = std::max(skew, static_cast<std::int64_t>(
                                std::llabs(leg.stamp.value - tick.value)));
    if (imu_ok)
      skew = std::max(skew, static_cast<std::int64_t>(
                                std::llabs(imu.stamp.value - tick.value)));

    out.skew_ns = skew;
    out.valid = (skew <= opt_.sync.max_skew_ns);

    if (!out.valid && opt_.sync.drop_invalid)
      continue;

    if (arm_ok && leg_ok && imu_ok) {
      fill_arm_leg_imu(out, arm, leg, imu);
    }

    frame_ring_.write([&](Frame &dst) { dst = out; });
    frame_seq_.fetch_add(1, std::memory_order_relaxed);
    frame_cv_.notify_all();
  }
}

// ---- command set/commit ----
static void require_size(std::span<const double> v, std::size_t expect,
                         std::string_view what) {
  if (v.size() != expect)
    throw std::invalid_argument(std::string(what) +
                                " size mismatch: " + std::to_string(v.size()) +
                                " != " + std::to_string(expect));
}

template <int DOF>
static std::array<double, DOF> &field_ref(PendingCommand<DOF> &pending,
                                          Field f, std::string_view what) {
  switch (f) {
    case Field::Position:
      return pending.pos;
    case Field::Velocity:
      return pending.vel;
    case Field::Effort:
      return pending.eff;
    case Field::Stiffness:
      return pending.kp;
    case Field::Damping:
      return pending.kd;
  }
  throw std::invalid_argument(std::string("invalid ") + std::string(what) +
                              " field");
}

void Core::set_arm(Field f, std::span<const double> v) {
  require_size(v, kArmDof, "arm field");
  std::lock_guard lk(cmd_mtx_);
  arm_pending_.has_any = true;
  arm_pending_.mask |= to_mask(f);

  auto &dst = field_ref(arm_pending_, f, "arm");
  std::ranges::copy(v, dst.begin());
}

void Core::set_leg(Field f, std::span<const double> v) {
  require_size(v, kLegDof, "leg field");
  std::lock_guard lk(cmd_mtx_);
  leg_pending_.has_any = true;
  leg_pending_.mask |= to_mask(f);

  auto &dst = field_ref(leg_pending_, f, "leg");
  std::ranges::copy(v, dst.begin());
}

void Core::set_arm_scalar(Field f, double scalar) {
  std::lock_guard lk(cmd_mtx_);
  arm_pending_.has_any = true;
  arm_pending_.mask |= to_mask(f);

  auto &dst = field_ref(arm_pending_, f, "arm");
  std::ranges::fill(dst, scalar);
}

void Core::set_leg_scalar(Field f, double scalar) {
  std::lock_guard lk(cmd_mtx_);
  leg_pending_.has_any = true;
  leg_pending_.mask |= to_mask(f);

  auto &dst = field_ref(leg_pending_, f, "leg");
  std::ranges::fill(dst, scalar);
}

void Core::commit(std::optional<TimestampNs> stamp,
                  std::optional<Sequence32> seq) {
  std::lock_guard lk(cmd_mtx_);
  const auto s = stamp.value_or(now_system_ns());
  const auto q = seq.value_or(Sequence32{++commit_seq_});

  transport_->publish_arm_command(s, q, arm_pending_, opt_.arm_names);
  transport_->publish_leg_command(s, q, leg_pending_, opt_.leg_names);
}

}  // namespace aimrl_sdk

#include "core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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

namespace {

using closed_ankle_detail::LoopAnkleAnalyticalSolver;
using closed_ankle_detail::Vec2;

bool read_latest_leg_motor_(const RingBuffer<JointSample<kLegDof>> &ring,
                            JointSample<kLegDof> &out) {
  auto idx = ring.latest_index();
  if (idx == 0)
    return false;
  for (int i = 0; i < 3 && idx > 0; ++i, --idx) {
    if (ring.read_at(idx, out))
      return true;
  }
  return false;
}

void apply_closed_ankle_state_(JointSample<kLegDof> &leg,
                               const ClosedAnkleParams &p) {
  // This mirrors `deploy/legged_system/src/LeggedSystem.cpp::processClosedChainState()`
  // but only for the 12-DOF legs (hip + tarsus + toe_pitch/toe_roll).
  thread_local LoopAnkleAnalyticalSolver solver;

  for (int leg_index = 0; leg_index < 2; ++leg_index) {
    const int base = leg_index * 6;
    const int idx_m0 = base + 4;  // motorA in /body_drive topic ordering
    const int idx_m1 = base + 5;  // motorB in /body_drive topic ordering

    closed_ankle_detail::solver_link_lengths_for_leg(solver, leg_index, p);

    const Vec2 mc_pos_actual{leg.pos[static_cast<std::size_t>(idx_m0)],
                             leg.pos[static_cast<std::size_t>(idx_m1)]};
    const Vec2 mc_vel_actual{leg.vel[static_cast<std::size_t>(idx_m0)],
                             leg.vel[static_cast<std::size_t>(idx_m1)]};
    const Vec2 mc_tau_actual{leg.eff[static_cast<std::size_t>(idx_m0)],
                             leg.eff[static_cast<std::size_t>(idx_m1)]};

    const Vec2 mc_pos = closed_ankle_detail::solver_mc_from_actual(leg_index, mc_pos_actual, p);
    const Vec2 mc_vel = closed_ankle_detail::solver_mc_from_actual(leg_index, mc_vel_actual, p);
    const Vec2 mc_tau = closed_ankle_detail::solver_mc_from_actual(leg_index, mc_tau_actual, p);

    Vec2 pr = solver.FK(mc_pos);  // physical sign
    pr.v0 = closed_ankle_detail::clamp_abs(pr.v0, p.pitch_limit);
    pr.v1 = closed_ankle_detail::clamp_abs(pr.v1, p.roll_limit);
    pr = closed_ankle_detail::apply_pr_direction(pr, p);  // controller sign

    solver.UpdateJ1(mc_pos);

    Vec2 vel_pr = solver.DFK(mc_vel);
    vel_pr = closed_ankle_detail::apply_pr_direction(vel_pr, p);

    Vec2 tau_pr = solver.FDyn(mc_tau);
    tau_pr = closed_ankle_detail::apply_pr_direction(tau_pr, p);

    leg.pos[static_cast<std::size_t>(idx_m0)] = pr.v0;   // toe_pitch
    leg.pos[static_cast<std::size_t>(idx_m1)] = pr.v1;   // toe_roll
    leg.vel[static_cast<std::size_t>(idx_m0)] = vel_pr.v0;
    leg.vel[static_cast<std::size_t>(idx_m1)] = vel_pr.v1;
    leg.eff[static_cast<std::size_t>(idx_m0)] = tau_pr.v0;
    leg.eff[static_cast<std::size_t>(idx_m1)] = tau_pr.v1;
  }
}

}  // namespace

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
      if (opt_.use_closed_ankle) {
        auto leg_joint = leg;  // convert motor-space ankle -> (pitch,roll)
        apply_closed_ankle_state_(leg_joint, opt_.closed_ankle);
        fill_arm_leg_imu(out, arm, leg_joint, imu);
      } else {
        fill_arm_leg_imu(out, arm, leg, imu);
      }
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
  PendingCommand<kArmDof> arm_cmd{};
  PendingCommand<kLegDof> leg_cmd{};
  TimestampNs s{};
  Sequence32 q{};
  {
    std::lock_guard lk(cmd_mtx_);
    s = stamp.value_or(now_system_ns());
    q = seq.value_or(Sequence32{++commit_seq_});
    arm_cmd = arm_pending_;
    leg_cmd = leg_pending_;
  }

  if (opt_.use_closed_ankle) {
    JointSample<kLegDof> leg_motor{};
    if (read_latest_leg_motor_(leg_raw_, leg_motor)) {
      thread_local LoopAnkleAnalyticalSolver solver;
      const auto &p = opt_.closed_ankle;

      for (int leg_index = 0; leg_index < 2; ++leg_index) {
        const int base = leg_index * 6;
        const int idx_m0 = base + 4;
        const int idx_m1 = base + 5;

        closed_ankle_detail::solver_link_lengths_for_leg(solver, leg_index, p);

        const Vec2 cur_mc_pos_actual{leg_motor.pos[static_cast<std::size_t>(idx_m0)],
                                     leg_motor.pos[static_cast<std::size_t>(idx_m1)]};
        const Vec2 cur_mc_vel_actual{leg_motor.vel[static_cast<std::size_t>(idx_m0)],
                                     leg_motor.vel[static_cast<std::size_t>(idx_m1)]};
        const Vec2 cur_mc_pos = closed_ankle_detail::solver_mc_from_actual(leg_index, cur_mc_pos_actual, p);
        const Vec2 cur_mc_vel = closed_ankle_detail::solver_mc_from_actual(leg_index, cur_mc_vel_actual, p);

        // current ankle state in controller sign
        Vec2 cur_pr = solver.FK(cur_mc_pos);
        cur_pr.v0 = closed_ankle_detail::clamp_abs(cur_pr.v0, p.pitch_limit);
        cur_pr.v1 = closed_ankle_detail::clamp_abs(cur_pr.v1, p.roll_limit);
        cur_pr = closed_ankle_detail::apply_pr_direction(cur_pr, p);

        solver.UpdateJ1(cur_mc_pos);

        Vec2 cur_vel_pr = solver.DFK(cur_mc_vel);
        cur_vel_pr = closed_ankle_detail::apply_pr_direction(cur_vel_pr, p);

        // desired ankle command in controller sign (reuse cmd arrays at indices 4/5 and 10/11)
        Vec2 des_pr{leg_cmd.pos[static_cast<std::size_t>(idx_m0)],
                    leg_cmd.pos[static_cast<std::size_t>(idx_m1)]};
        des_pr.v0 = closed_ankle_detail::clamp_abs(des_pr.v0, p.pitch_limit);
        des_pr.v1 = closed_ankle_detail::clamp_abs(des_pr.v1, p.roll_limit);

        Vec2 des_vel_pr{leg_cmd.vel[static_cast<std::size_t>(idx_m0)],
                        leg_cmd.vel[static_cast<std::size_t>(idx_m1)]};
        Vec2 ff_tau_pr{leg_cmd.eff[static_cast<std::size_t>(idx_m0)],
                       leg_cmd.eff[static_cast<std::size_t>(idx_m1)]};
        const Vec2 kp{leg_cmd.kp[static_cast<std::size_t>(idx_m0)],
                      leg_cmd.kp[static_cast<std::size_t>(idx_m1)]};
        const Vec2 kd{leg_cmd.kd[static_cast<std::size_t>(idx_m0)],
                      leg_cmd.kd[static_cast<std::size_t>(idx_m1)]};

        if (opt_.ankle_torque_control) {
          // PD in ankle (pitch/roll) space -> convert to motor torques.
          const Vec2 tau_pr_ctrl{
              ff_tau_pr.v0 + kp.v0 * (des_pr.v0 - cur_pr.v0) + kd.v0 * (des_vel_pr.v0 - cur_vel_pr.v0),
              ff_tau_pr.v1 + kp.v1 * (des_pr.v1 - cur_pr.v1) + kd.v1 * (des_vel_pr.v1 - cur_vel_pr.v1),
          };

          const Vec2 tau_pr_phys = closed_ankle_detail::remove_pr_direction(tau_pr_ctrl, p);
          const Vec2 tau_m_solver = solver.IDyn(tau_pr_phys);
          const Vec2 tau_m_actual = closed_ankle_detail::actual_mc_from_solver(leg_index, tau_m_solver, p);

          // For ankle motors, publish effort (torque) only (mirrors deploy's default behavior).
          leg_cmd.pos[static_cast<std::size_t>(idx_m0)] = 0.0;
          leg_cmd.pos[static_cast<std::size_t>(idx_m1)] = 0.0;
          leg_cmd.vel[static_cast<std::size_t>(idx_m0)] = 0.0;
          leg_cmd.vel[static_cast<std::size_t>(idx_m1)] = 0.0;
          leg_cmd.eff[static_cast<std::size_t>(idx_m0)] = tau_m_actual.v0;
          leg_cmd.eff[static_cast<std::size_t>(idx_m1)] = tau_m_actual.v1;
          leg_cmd.kp[static_cast<std::size_t>(idx_m0)] = 0.0;
          leg_cmd.kp[static_cast<std::size_t>(idx_m1)] = 0.0;
          leg_cmd.kd[static_cast<std::size_t>(idx_m0)] = 0.0;
          leg_cmd.kd[static_cast<std::size_t>(idx_m1)] = 0.0;
        } else {
          // Position-space conversion (pitch/roll -> motor angles). Gains are not mapped.
          const Vec2 des_pr_phys = closed_ankle_detail::remove_pr_direction(des_pr, p);
          const Vec2 des_mc_solver = solver.IK(des_pr_phys);
          Vec2 des_mc_actual = closed_ankle_detail::actual_mc_from_solver(leg_index, des_mc_solver, p);
          des_mc_actual.v0 = closed_ankle_detail::clamp_abs(des_mc_actual.v0, p.actuator_pos_limit);
          des_mc_actual.v1 = closed_ankle_detail::clamp_abs(des_mc_actual.v1, p.actuator_pos_limit);

          leg_cmd.pos[static_cast<std::size_t>(idx_m0)] = des_mc_actual.v0;
          leg_cmd.pos[static_cast<std::size_t>(idx_m1)] = des_mc_actual.v1;
          leg_cmd.vel[static_cast<std::size_t>(idx_m0)] = 0.0;
          leg_cmd.vel[static_cast<std::size_t>(idx_m1)] = 0.0;
          leg_cmd.eff[static_cast<std::size_t>(idx_m0)] = 0.0;
          leg_cmd.eff[static_cast<std::size_t>(idx_m1)] = 0.0;

          // Map (pitch,roll) PD gains to motor-space gains using current Jacobian:
          //   K_m ~= J^T * K_pr * J   (diagonal approximation).
          const auto col0 = solver.DFK(Vec2{1.0, 0.0});
          const auto col1 = solver.DFK(Vec2{0.0, 1.0});
          auto kp_m0_solver = kp.v0 * col0.v0 * col0.v0 + kp.v1 * col0.v1 * col0.v1;
          auto kp_m1_solver = kp.v0 * col1.v0 * col1.v0 + kp.v1 * col1.v1 * col1.v1;
          auto kd_m0_solver = kd.v0 * col0.v0 * col0.v0 + kd.v1 * col0.v1 * col0.v1;
          auto kd_m1_solver = kd.v0 * col1.v0 * col1.v0 + kd.v1 * col1.v1 * col1.v1;
          if (!std::isfinite(kp_m0_solver) || kp_m0_solver < 0.0)
            kp_m0_solver = 0.0;
          if (!std::isfinite(kp_m1_solver) || kp_m1_solver < 0.0)
            kp_m1_solver = 0.0;
          if (!std::isfinite(kd_m0_solver) || kd_m0_solver < 0.0)
            kd_m0_solver = 0.0;
          if (!std::isfinite(kd_m1_solver) || kd_m1_solver < 0.0)
            kd_m1_solver = 0.0;

          const double kp_actual0 = (leg_index == 0) ? kp_m1_solver : kp_m0_solver;
          const double kp_actual1 = (leg_index == 0) ? kp_m0_solver : kp_m1_solver;
          const double kd_actual0 = (leg_index == 0) ? kd_m1_solver : kd_m0_solver;
          const double kd_actual1 = (leg_index == 0) ? kd_m0_solver : kd_m1_solver;
          leg_cmd.kp[static_cast<std::size_t>(idx_m0)] = kp_actual0;
          leg_cmd.kp[static_cast<std::size_t>(idx_m1)] = kp_actual1;
          leg_cmd.kd[static_cast<std::size_t>(idx_m0)] = kd_actual0;
          leg_cmd.kd[static_cast<std::size_t>(idx_m1)] = kd_actual1;
        }
      }
    }
  }

  transport_->publish_arm_command(s, q, arm_cmd, opt_.arm_names);
  transport_->publish_leg_command(s, q, leg_cmd, opt_.leg_names);
}

}  // namespace aimrl_sdk

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <dlfcn.h>

#include "aimrt_transport.hpp"
#include "core.hpp"
#include "layout.hpp"

namespace py = pybind11;
using namespace aimrl_sdk;

namespace {

std::string g_module_dir;
std::weak_ptr<Core> g_last_core;

void init_module_dir(const py::module_ &m) {
  try {
    if (py::hasattr(m, "__file__")) {
      const auto module_file = py::str(m.attr("__file__"));
      g_module_dir = std::filesystem::path(module_file)
                         .parent_path()
                         .string();
    }
  } catch (const std::exception &) {
  }

  if (g_module_dir.empty()) {
    Dl_info info;
    if (dladdr(reinterpret_cast<void *>(&init_module_dir), &info) != 0 &&
        info.dli_fname != nullptr) {
      g_module_dir =
          std::filesystem::path(info.dli_fname).parent_path().string();
    }
  }
}

std::string default_config_path() {
  if (const char *env = std::getenv("AIMRL_SDK_CONFIG"); env && *env) {
    return std::string(env);
  }
  if (!g_module_dir.empty()) {
    const auto cfg_dir = std::filesystem::path(g_module_dir) / "config";
    const auto iceoryx_candidate = cfg_dir / "aimrt_iceoryx_backend.yaml";
    if (std::filesystem::exists(iceoryx_candidate))
      return iceoryx_candidate.string();

    const auto ros2_candidate = cfg_dir / "aimrt_ros2_backend.yaml";
    if (std::filesystem::exists(ros2_candidate))
      return ros2_candidate.string();
  }
  // Default to iceoryx backend; ROS2 backend can be selected via AIMRL_SDK_CONFIG.
  return "config/aimrt_iceoryx_backend.yaml";
}

std::span<const double> as_span_double(py::array &arr,
                                       std::vector<double> &owned) {
  // allow float32/float64; convert to double contiguous for safety
  if (arr.ndim() != 1)
    throw std::invalid_argument("array must be 1D");
  const auto n = static_cast<std::size_t>(arr.shape(0));
  owned.resize(n);

  if (py::isinstance<py::array_t<double>>(arr)) {
    auto a = py::array_t < double,
         py::array::c_style | py::array::forcecast > (arr);
    std::memcpy(owned.data(), a.data(), sizeof(double) * n);
  } else {
    auto a = py::array_t < float,
         py::array::c_style | py::array::forcecast > (arr);
    for (std::size_t i = 0; i < n; ++i)
      owned[i] = static_cast<double>(a.data()[i]);
  }
  return {owned.data(), owned.size()};
}

std::optional<double> parse_timeout(const py::object &o) {
  if (o.is_none())
    return std::nullopt;
  return py::cast<double>(o);
}

std::string resolve_config_path(const py::object &config_path) {
  if (config_path.is_none())
    return default_config_path();
  auto s = py::cast<std::string>(config_path);
  if (s.empty())
    return default_config_path();
  return s;
}

std::vector<std::string> default_arm_names() {
  return {
      "idx13_left_arm_joint1",
      "idx14_left_arm_joint2",
      "idx15_left_arm_joint3",
      "idx16_left_arm_joint4",
      "idx17_left_arm_joint5",
      "idx18_01_left_wrist_rod_A_joint",
      "idx19_01_left_wrist_rod_B_joint",
      "idx20_right_arm_joint1",
      "idx21_right_arm_joint2",
      "idx22_right_arm_joint3",
      "idx23_right_arm_joint4",
      "idx24_right_arm_joint5",
      "idx25_01_right_wrist_rod_A_joint",
      "idx26_01_right_wrist_rod_B_joint"};
}

std::vector<std::string> default_leg_names() {
  return {
      "idx01_left_hip_roll",
      "idx02_left_hip_yaw",
      "idx03_left_hip_pitch",
      "idx04_left_tarsus",
      "idx05_01_left_toe_motorA",
      "idx06_01_left_toe_motorB",
      "idx07_right_hip_roll",
      "idx08_right_hip_yaw",
      "idx09_right_hip_pitch",
      "idx10_right_tarsus",
      "idx11_01_right_toe_motorA",
      "idx12_01_right_toe_motorB"};
}

py::dict metric_to_dict(const StatsMetricSnapshot &m) {
  py::dict d;
  d["count"] = py::int_(m.count);
  d["last_ns"] = py::int_(m.last);
  d["ema_ns"] = py::int_(m.ema);
  d["min_ns"] = py::int_(m.min);
  d["max_ns"] = py::int_(m.max);
  return d;
}

py::dict stream_to_dict(const StreamStatsSnapshot &s) {
  py::dict d;
  d["rx_total"] = py::int_(s.rx_total);
  d["rx_stamp_missing"] = py::int_(s.rx_stamp_missing);
  d["rx_negative_delay"] = py::int_(s.rx_negative_delay);
  d["delay_ns"] = metric_to_dict(s.delay_ns);
  d["interval_ns"] = metric_to_dict(s.interval_ns);
  d["interval_jitter_ns"] = metric_to_dict(s.interval_jitter_ns);
  return d;
}

py::dict publish_to_dict(const PublishStatsSnapshot &p) {
  py::dict d;
  d["attempts"] = py::int_(p.attempts);
  d["skipped_no_cmd"] = py::int_(p.skipped_no_cmd);
  d["duration_ns"] = metric_to_dict(p.duration_ns);
  return d;
}

py::dict sync_to_dict(const SyncStatsSnapshot &s) {
  py::dict d;
  d["tick_total"] = py::int_(s.tick_total);
  d["tick_overrun"] = py::int_(s.tick_overrun);
  d["wake_lateness_ns"] = metric_to_dict(s.wake_lateness_ns);
  d["compute_ns"] = metric_to_dict(s.compute_ns);
  d["age_arm_ns"] = metric_to_dict(s.age_arm_ns);
  d["age_leg_ns"] = metric_to_dict(s.age_leg_ns);
  d["age_imu_ns"] = metric_to_dict(s.age_imu_ns);
  d["missing_arm"] = py::int_(s.missing_arm);
  d["missing_leg"] = py::int_(s.missing_leg);
  d["missing_imu"] = py::int_(s.missing_imu);
  d["frame_written"] = py::int_(s.frame_written);
  d["frame_complete"] = py::int_(s.frame_complete);
  d["frame_incomplete"] = py::int_(s.frame_incomplete);
  d["frame_aligned"] = py::int_(s.frame_aligned);
  d["frame_unaligned_skew"] = py::int_(s.frame_unaligned_skew);
  d["frame_incomplete_missing_arm"] = py::int_(s.frame_incomplete_missing_arm);
  d["frame_incomplete_missing_leg"] = py::int_(s.frame_incomplete_missing_leg);
  d["frame_incomplete_missing_imu"] = py::int_(s.frame_incomplete_missing_imu);
  return d;
}

py::dict wait_frame_to_dict(const WaitFrameStatsSnapshot &w) {
  py::dict d;
  d["calls"] = py::int_(w.calls);
  d["ok"] = py::int_(w.ok);
  d["timeout"] = py::int_(w.timeout);
  d["stopped"] = py::int_(w.stopped);
  d["wait_ns"] = metric_to_dict(w.wait_ns);
  return d;
}

py::dict statistics_to_dict(const StatisticsSnapshot &s) {
  py::dict d;
  d["enabled"] = py::bool_(s.enabled);
  d["sample_every"] = py::int_(s.sample_every);
  d["ema_shift"] = py::int_(s.ema_shift);
  d["now_steady_ns"] = py::int_(s.now_steady_ns);
  d["start_steady_ns"] = py::int_(s.start_steady_ns);
  d["uptime_ns"] = py::int_(s.now_steady_ns - s.start_steady_ns);
  d["arm_state"] = stream_to_dict(s.arm_state);
  d["leg_state"] = stream_to_dict(s.leg_state);
  d["imu"] = stream_to_dict(s.imu);
  d["publish_arm"] = publish_to_dict(s.publish_arm);
  d["publish_leg"] = publish_to_dict(s.publish_leg);
  d["commit_total"] = publish_to_dict(s.commit_total);
  d["sync"] = sync_to_dict(s.sync);
  d["wait_frame"] = wait_frame_to_dict(s.wait_frame);
  return d;
}

}  // namespace

struct PyState {
  std::shared_ptr<Core> core;
  std::uint64_t last_seq{0};
};

struct PyCmd {
  std::shared_ptr<Core> core;
};

PYBIND11_MODULE(_bindings, m) {
  init_module_dir(m);

  // ---- observation layout constants (single source of truth) ----
  m.attr("ARM_DOF") = py::int_(kArmDof);
  m.attr("LEG_DOF") = py::int_(kLegDof);
  m.attr("FRAME_DIM") = py::int_(kFrameDim);
  m.attr("ARM_POS0") = py::int_(FrameLayout::ArmPos0);
  m.attr("ARM_VEL0") = py::int_(FrameLayout::ArmVel0);
  m.attr("ARM_EFF0") = py::int_(FrameLayout::ArmEff0);
  m.attr("LEG_POS0") = py::int_(FrameLayout::LegPos0);
  m.attr("LEG_VEL0") = py::int_(FrameLayout::LegVel0);
  m.attr("LEG_EFF0") = py::int_(FrameLayout::LegEff0);
  m.attr("IMU_QUAT0") = py::int_(FrameLayout::ImuQuat0);
  m.attr("IMU_GYRO0") = py::int_(FrameLayout::ImuGyro0);
  m.attr("IMU_ACC0") = py::int_(FrameLayout::ImuAcc0);

  py::class_<PyState>(m, "StateInterface")
      .def("latest_frame",
           [](PyState &self) {
             auto f = self.core->latest_frame();
             if (!f) {
               py::array_t<float> x(kFrameDim);
               std::memset(x.mutable_data(), 0, sizeof(float) * kFrameDim);
               return py::make_tuple(std::int64_t{0}, false, false, x);
             }
             py::array_t<float> x(kFrameDim);
             std::memcpy(x.mutable_data(), f->x.data(),
                         sizeof(float) * kFrameDim);
             return py::make_tuple(f->stamp.value, f->aligned, f->complete, x);
           })
      .def(
          "wait_frame",
          [](PyState &self, const py::object &timeout_s) {
            const auto after = self.core->frame_seq();
            py::gil_scoped_release release;

            auto r = self.core->wait_next_frame_ex(after, parse_timeout(timeout_s));
            if (!r.frame) {
              if (r.status == WaitFrameStatus::Timeout)
                throw std::runtime_error("wait_frame timeout");
              throw std::runtime_error("wait_frame stopped");
            }

            py::gil_scoped_acquire acquire;
            py::array_t<float> x(kFrameDim);
            std::memcpy(x.mutable_data(), r.frame->x.data(),
                        sizeof(float) * kFrameDim);
            return py::make_tuple(r.frame->stamp.value, r.frame->aligned,
                                  r.frame->complete, x);
          },
          py::arg("timeout_s") = py::none())
      .def("statistics",
           [](PyState &self) { return statistics_to_dict(self.core->statistics_snapshot()); })
      .def("reset_statistics", [](PyState &self) { self.core->reset_statistics(); })
      .def(
          "configure_statistics",
          [](PyState &self, bool enabled, std::uint32_t sample_every, int ema_shift) {
            self.core->set_statistics_config(
                Statistics::Config{.enabled = enabled,
                                   .sample_every = sample_every,
                                   .ema_shift = ema_shift});
          },
          py::arg("enabled"), py::arg("sample_every") = 1,
          py::arg("ema_shift") = 4)
      .def("read_frames", [](PyState &self, int n) {
        auto frames = self.core->read_last_frames(n);

        py::array_t<std::int64_t> stamps(n);
        py::array_t<std::uint8_t> aligneds(n);
        py::array_t<std::uint8_t> completes(n);
        py::array_t<float> X({n, kFrameDim});

        auto *sp = stamps.mutable_data();
        auto *ap = aligneds.mutable_data();
        auto *cp = completes.mutable_data();
        auto *xp = X.mutable_data();

        for (int i = 0; i < n; ++i) {
          const auto &f = frames[static_cast<std::size_t>(i)];
          sp[i] = f.stamp.value;
          ap[i] = static_cast<std::uint8_t>(f.aligned);
          cp[i] = static_cast<std::uint8_t>(f.complete);
          std::memcpy(xp + static_cast<std::size_t>(i) * kFrameDim, f.x.data(),
                      sizeof(float) * kFrameDim);
        }
        return py::make_tuple(stamps, aligneds, completes, X);
      });

  py::class_<PyCmd>(m, "CommandInterface")
      .def("statistics",
           [](PyCmd &self) { return statistics_to_dict(self.core->statistics_snapshot()); })
      .def("reset_statistics", [](PyCmd &self) { self.core->reset_statistics(); })
      .def(
          "configure_statistics",
          [](PyCmd &self, bool enabled, std::uint32_t sample_every, int ema_shift) {
            self.core->set_statistics_config(
                Statistics::Config{.enabled = enabled,
                                   .sample_every = sample_every,
                                   .ema_shift = ema_shift});
          },
          py::arg("enabled"), py::arg("sample_every") = 1,
          py::arg("ema_shift") = 4)
      .def(
          "set_arm",
          [](PyCmd &self, const py::object &pos, const py::object &vel, const py::object &eff,
             const py::object &kp, const py::object &kd) {
            auto core = self.core;
            auto handle = [&](const py::object &o, Field f) {
              if (o.is_none())
                return;
              if (py::isinstance<py::float_>(o) ||
                  py::isinstance<py::int_>(o)) {
                core->set_arm_scalar(f, py::cast<double>(o));
                return;
              }
              py::array arr = py::array::ensure(o);
              std::vector<double> owned;
              core->set_arm(f, as_span_double(arr, owned));
            };
            handle(pos, Field::Position);
            handle(vel, Field::Velocity);
            handle(eff, Field::Effort);
            handle(kp, Field::Stiffness);
            handle(kd, Field::Damping);
          },
          py::arg("position") = py::none(), py::arg("velocity") = py::none(),
          py::arg("effort") = py::none(), py::arg("stiffness") = py::none(),
          py::arg("damping") = py::none())

      .def(
          "set_leg",
          [](PyCmd &self, const py::object &pos, const py::object &vel, const py::object &eff,
             const py::object &kp, const py::object &kd) {
            auto core = self.core;
            auto handle = [&](const py::object &o, Field f) {
              if (o.is_none())
                return;
              if (py::isinstance<py::float_>(o) ||
                  py::isinstance<py::int_>(o)) {
                core->set_leg_scalar(f, py::cast<double>(o));
                return;
              }
              py::array arr = py::array::ensure(o);
              std::vector<double> owned;
              core->set_leg(f, as_span_double(arr, owned));
            };
            handle(pos, Field::Position);
            handle(vel, Field::Velocity);
            handle(eff, Field::Effort);
            handle(kp, Field::Stiffness);
            handle(kd, Field::Damping);
          },
          py::arg("position") = py::none(), py::arg("velocity") = py::none(),
          py::arg("effort") = py::none(), py::arg("stiffness") = py::none(),
          py::arg("damping") = py::none())

      .def(
          "commit",
          [](PyCmd &self, const py::object &stamp_ns, const py::object &seq) {
            std::optional<TimestampNs> s;
            std::optional<Sequence32> q;
            if (!stamp_ns.is_none())
              s = TimestampNs{py::cast<std::int64_t>(stamp_ns)};
            if (!seq.is_none())
              q = Sequence32{py::cast<std::uint32_t>(seq)};
            self.core->commit(s, q);
          },
          py::arg("stamp_ns") = py::none(), py::arg("sequence") = py::none());

  m.def(
      "open",
      [](const py::object &config_path, double sync_hz, double max_skew_ms,
         int max_backtrack, std::uint32_t raw_ring, std::uint32_t frame_ring,
         const py::object &arm_names, const py::object &leg_names,
         bool use_closed_ankle, bool ankle_torque_control,
         int ankle_motor1_direction, int ankle_motor2_direction,
         int ankle_pitch_direction, int ankle_roll_direction, double ankle_d,
         double ankle_l, double ankle_h1, double ankle_h2,
         double ankle_actuator_pos_limit, double ankle_pitch_limit,
         double ankle_roll_limit, bool enable_statistics,
         std::uint32_t statistics_sample_every, int statistics_ema_shift) {
        Core::Options opt;
        opt.raw_ring = raw_ring;
        opt.frame_ring = frame_ring;

        opt.sync.frame_hz = sync_hz;
        if (!std::isfinite(max_skew_ms) || max_skew_ms < 0.0) {
          throw std::invalid_argument("max_skew_ms must be >= 0");
        }
        const auto max_skew_ns_f = max_skew_ms * 1e6;
        if (!std::isfinite(max_skew_ns_f) ||
            max_skew_ns_f >
                static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
          throw std::invalid_argument("max_skew_ms out of range");
        }
        opt.sync.max_skew_ns = static_cast<std::int64_t>(max_skew_ns_f);
        opt.sync.max_backtrack = max_backtrack;

        opt.use_closed_ankle = use_closed_ankle;
        opt.ankle_torque_control = ankle_torque_control;
        opt.closed_ankle.motor1_direction = ankle_motor1_direction;
        opt.closed_ankle.motor2_direction = ankle_motor2_direction;
        opt.closed_ankle.pitch_direction = ankle_pitch_direction;
        opt.closed_ankle.roll_direction = ankle_roll_direction;
        opt.closed_ankle.d = ankle_d;
        opt.closed_ankle.l = ankle_l;
        opt.closed_ankle.h1 = ankle_h1;
        opt.closed_ankle.h2 = ankle_h2;
        opt.closed_ankle.actuator_pos_limit = ankle_actuator_pos_limit;
        opt.closed_ankle.pitch_limit = ankle_pitch_limit;
        opt.closed_ankle.roll_limit = ankle_roll_limit;

        opt.arm_names = default_arm_names();
        opt.leg_names = default_leg_names();
        if (!arm_names.is_none())
          opt.arm_names = py::cast<std::vector<std::string>>(arm_names);
        if (!leg_names.is_none())
          opt.leg_names = py::cast<std::vector<std::string>>(leg_names);

        opt.statistics.enabled = enable_statistics;
        opt.statistics.sample_every = statistics_sample_every;
        opt.statistics.ema_shift = statistics_ema_shift;

        const auto cfg = resolve_config_path(config_path);
        auto core = std::make_shared<Core>(
            opt, std::make_unique<aimrl_sdk::AimrtTransport>(cfg));
        core->start();
        g_last_core = core;

        PyState st{.core = core};
        PyCmd cmd{.core = core};
        return py::make_tuple(st, cmd);
      },
      R"pbdoc(
Open the AimRL SDK and return `(state, cmd)`.

  Args:
  config_path: Optional path to the AimRT backend YAML. If None/empty, uses
    `AIMRL_SDK_CONFIG` (if set) or the built-in default.
  sync_hz: Frame synchronization frequency (Hz) for generating aligned frames.
  max_skew_ms: Max allowed timestamp skew (ms) for a frame to be marked `aligned`.
  max_backtrack: Max samples to scan backward per tick to find `<= tick` samples.
  raw_ring: Raw sample ring capacity (arm/leg/imu).
  frame_ring: Aligned frame ring capacity.
  arm_names: Optional list[str] of length 14 for command joint names.
  leg_names: Optional list[str] of length 12 for command joint names.
  use_closed_ankle: If True, convert the ankle closed-chain motors (toe A/B)
    to ankle (pitch,roll) for frames, and convert commands back to motors.
  ankle_torque_control: If True and `use_closed_ankle`, ankle motors are
    commanded in effort (torque) based on (pitch,roll) PD in `commit()`.
  ankle_*: Closed-chain ankle geometry/sign parameters (advanced).
  enable_statistics: Enable lightweight runtime statistics counters/latency/jitter.
  statistics_sample_every: Sample 1/N events when aggregating statistics.
  statistics_ema_shift: EMA smoothing shift (alpha = 1/2^shift).
)pbdoc",
      py::arg("config_path") = py::none(), py::arg("sync_hz") = 100.0,
      py::arg("max_skew_ms") = 3.0, py::arg("max_backtrack") = 200,
      py::arg("raw_ring") = 2048, py::arg("frame_ring") = 512,
      py::arg("arm_names") = py::none(), py::arg("leg_names") = py::none(),
      py::arg("use_closed_ankle") = true,
      py::arg("ankle_torque_control") = true,
      py::arg("ankle_motor1_direction") = 1,
      py::arg("ankle_motor2_direction") = 1, py::arg("ankle_pitch_direction") = 1,
      py::arg("ankle_roll_direction") = 1, py::arg("ankle_d") = 0.0315,
      py::arg("ankle_l") = 0.063, py::arg("ankle_h1") = 0.239,
      py::arg("ankle_h2") = 0.145, py::arg("ankle_actuator_pos_limit") = 1.0,
      py::arg("ankle_pitch_limit") = 1.0, py::arg("ankle_roll_limit") = 0.5,
      py::arg("enable_statistics") = false,
      py::arg("statistics_sample_every") = 1,
      py::arg("statistics_ema_shift") = 4);

  m.def("close", [](const py::object &handle) {
    std::shared_ptr<Core> core;
    if (handle.is_none()) {
      core = g_last_core.lock();
    } else if (py::isinstance<PyState>(handle)) {
      core = py::cast<PyState &>(handle).core;
    } else if (py::isinstance<PyCmd>(handle)) {
      core = py::cast<PyCmd &>(handle).core;
    } else {
      throw std::invalid_argument("close expects a StateInterface, CommandInterface, or None");
    }
    if (core)
      core->stop();
  }, py::arg("handle") = py::none());
}

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdlib>
#include <filesystem>
#if !defined(_WIN32)
  #include <dlfcn.h>
#else
  #include <windows.h>
#endif

#include "aimrt_transport.hpp"
#include "core.hpp"

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
#if defined(_WIN32)
    HMODULE hm = nullptr;
    if (GetModuleHandleExA(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(&init_module_dir), &hm)) {
      char path[MAX_PATH];
      if (GetModuleFileNameA(hm, path, MAX_PATH) > 0) {
        g_module_dir = std::filesystem::path(path).parent_path().string();
      }
    }
#else
    Dl_info info;
    if (dladdr(reinterpret_cast<void *>(&init_module_dir), &info) != 0 &&
        info.dli_fname != nullptr) {
      g_module_dir =
          std::filesystem::path(info.dli_fname).parent_path().string();
    }
#endif
  }
}

std::string default_config_path() {
  if (const char *env = std::getenv("AIMRL_SDK_CONFIG"); env && *env) {
    return std::string(env);
  }
  if (!g_module_dir.empty()) {
    const auto candidate = std::filesystem::path(g_module_dir) / "config" /
                           "aimrt_ros2_backend.yaml";
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }
  }
  return "config/aimrt_ros2_backend.yaml";
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

  py::class_<PyState>(m, "StateInterface")
      .def("latest_frame",
           [](PyState &self) {
             auto f = self.core->latest_frame();
             if (!f) {
               py::array_t<float> x(kFrameDim);
               std::memset(x.mutable_data(), 0, sizeof(float) * kFrameDim);
               return py::make_tuple(std::int64_t{0}, false, x);
             }
             py::array_t<float> x(kFrameDim);
             std::memcpy(x.mutable_data(), f->x.data(),
                         sizeof(float) * kFrameDim);
             return py::make_tuple(f->stamp.value, f->valid, x);
           })
      .def(
          "wait_frame",
          [](PyState &self, const py::object &timeout_s) {
            const auto after = self.core->frame_seq();
            py::gil_scoped_release release;

            auto f =
                self.core->wait_next_frame(after, parse_timeout(timeout_s));
            if (!f)
              throw std::runtime_error("wait_frame timeout or stopped");

            py::gil_scoped_acquire acquire;
            py::array_t<float> x(kFrameDim);
            std::memcpy(x.mutable_data(), f->x.data(),
                        sizeof(float) * kFrameDim);
            return py::make_tuple(f->stamp.value, f->valid, x);
          },
          py::arg("timeout_s") = py::none())
      .def("read_frames", [](PyState &self, int n) {
        auto frames = self.core->read_last_frames(n);

        py::array_t<std::int64_t> stamps(n);
        py::array_t<std::uint8_t> valids(n);
        py::array_t<float> X({n, kFrameDim});

        auto *sp = stamps.mutable_data();
        auto *vp = valids.mutable_data();
        auto *xp = X.mutable_data();

        for (int i = 0; i < n; ++i) {
          const auto &f = frames[static_cast<std::size_t>(i)];
          sp[i] = f.stamp.value;
          vp[i] = static_cast<std::uint8_t>(f.valid);
          std::memcpy(xp + static_cast<std::size_t>(i) * kFrameDim, f.x.data(),
                      sizeof(float) * kFrameDim);
        }
        return py::make_tuple(stamps, valids, X);
      });

  py::class_<PyCmd>(m, "CommandInterface")
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

  m.def("open", []() {
    Core::Options opt;
    opt.arm_names = {
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
    opt.leg_names = {
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

    auto core = std::make_shared<Core>(
        opt, std::make_unique<aimrl_sdk::AimrtTransport>(default_config_path()));
    core->start();
    g_last_core = core;

    PyState st{.core = core};
    PyCmd cmd{.core = core};
    return py::make_tuple(st, cmd);
  });

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

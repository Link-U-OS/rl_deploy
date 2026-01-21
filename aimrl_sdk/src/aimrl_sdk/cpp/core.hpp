#pragma once
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>  // jthread
#include <vector>

#include "closed_ankle.hpp"
#include "ring_buffer.hpp"
#include "statistics.hpp"
#include "types.hpp"

#include "joint_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace aimrl_sdk {

class Transport {
 public:
  virtual ~Transport() = default;

  struct Callbacks {
    std::function<void(const std::shared_ptr<const joint_msgs::msg::JointState> &)>
        on_arm_state;
    std::function<void(const std::shared_ptr<const joint_msgs::msg::JointState> &)>
        on_leg_state;
    std::function<void(const std::shared_ptr<const sensor_msgs::msg::Imu> &)>
        on_imu;
  };

  // lifecycle
  virtual void start(Callbacks callbacks) = 0;
  virtual void stop() = 0;

  // publish (arm/leg JointCommand)
  // TODO: use template to reduce virtual function calls
  virtual void publish_arm_command(TimestampNs stamp, Sequence32 seq,
                                   const PendingCommand<kArmDof> &cmd,
                                   std::span<const std::string> arm_names) = 0;
  virtual void publish_leg_command(TimestampNs stamp, Sequence32 seq,
                                   const PendingCommand<kLegDof> &cmd,
                                   std::span<const std::string> leg_names) = 0;
};

class Core final {
 public:
  struct Options {
    std::uint32_t raw_ring{2048};
    std::uint32_t frame_ring{512};
    SyncConfig sync{};
    bool use_closed_ankle{true};
    bool ankle_torque_control{true};
    ClosedAnkleParams closed_ankle{};
    Statistics::Config statistics{};
    // internal fixed order name (not exposed to Python)
    std::vector<std::string> arm_names;  // size=14
    std::vector<std::string> leg_names;  // size=12
  };

  Core(Options opt, std::unique_ptr<Transport> transport);
  ~Core();

  Core(const Core &) = delete;
  Core &operator=(const Core &) = delete;
  Core(Core &&) = delete;
  Core &operator=(Core &&) = delete;

  void start();
  void stop();
  bool running() const noexcept {
    return running_.load(std::memory_order_relaxed);
  }

  // ---- callbacks from transport subscriber ----
  void on_arm_state(const std::shared_ptr<const joint_msgs::msg::JointState> &msg);
  void on_leg_state(const std::shared_ptr<const joint_msgs::msg::JointState> &msg);
  void on_imu(const std::shared_ptr<const sensor_msgs::msg::Imu> &msg);

  // ---- frame read API (for pybind wrappers) ----
  // latest (may repeat)
  std::optional<Frame> latest_frame() const;

  // wait for a new frame after given sequence; returns nullopt on timeout/stop
  std::optional<Frame> wait_next_frame(std::uint64_t after_seq,
                                       std::optional<double> timeout_s);

  struct WaitNextFrameResult {
    WaitFrameStatus status{WaitFrameStatus::Stopped};
    std::optional<Frame> frame{};
  };
  WaitNextFrameResult wait_next_frame_ex(std::uint64_t after_seq,
                                         std::optional<double> timeout_s);

  // read last n frames (oldest->newest). if not enough, pad leading default
  // frames.
  std::vector<Frame> read_last_frames(int n) const;

  std::uint64_t frame_seq() const noexcept {
    return frame_seq_.load(std::memory_order_relaxed);
  }

  // ---- command API ----
  void set_arm(Field f, std::span<const double> v);  // v.size==14
  void set_leg(Field f, std::span<const double> v);  // v.size==12
  void set_arm_scalar(Field f, double scalar);
  void set_leg_scalar(Field f, double scalar);

  void commit(std::optional<TimestampNs> stamp, std::optional<Sequence32> seq);

  void set_statistics_config(Statistics::Config cfg) noexcept;
  Statistics::Config statistics_config() const noexcept;
  void reset_statistics() noexcept;
  StatisticsSnapshot statistics_snapshot() const noexcept;

 private:
  void sync_loop_(const std::stop_token &stoken);

  // find sample <= t (linear backtrack)
  template <class Ring, class Sample>
  static bool find_leq_(const Ring &ring, TimestampNs t,
                        std::uint64_t start_idx, int max_backtrack,
                        Sample &out);

 private:
  Options opt_;
  std::unique_ptr<Transport> transport_;

  // raw rings
  RingBuffer<JointSample<kArmDof>> arm_raw_;
  RingBuffer<JointSample<kLegDof>> leg_raw_;
  RingBuffer<ImuSample> imu_raw_;

  // frame ring (store Frame directly)
  RingBuffer<Frame> frame_ring_;
  std::atomic<std::uint64_t> frame_seq_{0};

  // sync thread
  std::atomic<bool> running_{false};
  std::jthread sync_thread_;

  // waiters
  mutable std::mutex frame_mtx_;
  mutable std::condition_variable frame_cv_;

  // pending commands + publish atomicity
  std::mutex cmd_mtx_;
  PendingCommand<kArmDof> arm_pending_{};
  PendingCommand<kLegDof> leg_pending_{};
  std::uint32_t commit_seq_{0};

  Statistics stats_{};
};

}  // namespace aimrl_sdk

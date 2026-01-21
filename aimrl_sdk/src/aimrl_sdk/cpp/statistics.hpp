#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

namespace aimrl_sdk {

namespace stats_detail {

inline void atomic_min(std::atomic<std::int64_t> &dst,
                       std::int64_t v) noexcept {
  auto cur = dst.load(std::memory_order_relaxed);
  while (v < cur &&
         !dst.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
  }
}

inline void atomic_max(std::atomic<std::int64_t> &dst,
                       std::int64_t v) noexcept {
  auto cur = dst.load(std::memory_order_relaxed);
  while (v > cur &&
         !dst.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
  }
}

}  // namespace stats_detail

struct StatsMetricSnapshot {
  std::uint64_t count{0};  // sampled count
  std::int64_t last{0};
  std::int64_t ema{0};
  std::int64_t min{0};
  std::int64_t max{0};
};

class StatsMetric final {
 public:
  void reset() noexcept {
    count_.store(0, std::memory_order_relaxed);
    last_.store(0, std::memory_order_relaxed);
    ema_.store(0, std::memory_order_relaxed);
    min_.store(std::numeric_limits<std::int64_t>::max(),
               std::memory_order_relaxed);
    max_.store(std::numeric_limits<std::int64_t>::min(),
               std::memory_order_relaxed);
  }

  void add(std::int64_t v, int ema_shift) noexcept {
    last_.store(v, std::memory_order_relaxed);

    const auto new_count =
        count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (new_count == 1) {
      ema_.store(v, std::memory_order_relaxed);
    } else if (ema_shift <= 0) {
      ema_.store(v, std::memory_order_relaxed);
    } else {
      const auto denom = (std::int64_t{1} << ema_shift);
      const auto prev = ema_.load(std::memory_order_relaxed);
      const auto delta = v - prev;
      ema_.store(prev + delta / denom, std::memory_order_relaxed);
    }

    stats_detail::atomic_min(min_, v);
    stats_detail::atomic_max(max_, v);
  }

  StatsMetricSnapshot snapshot() const noexcept {
    StatsMetricSnapshot out;
    out.count = count_.load(std::memory_order_relaxed);
    out.last = last_.load(std::memory_order_relaxed);
    out.ema = ema_.load(std::memory_order_relaxed);
    out.min = min_.load(std::memory_order_relaxed);
    out.max = max_.load(std::memory_order_relaxed);
    if (out.count == 0) {
      out.min = 0;
      out.max = 0;
    }
    return out;
  }

  std::int64_t ema_relaxed() const noexcept {
    return ema_.load(std::memory_order_relaxed);
  }

 private:
  std::atomic<std::uint64_t> count_{0};
  std::atomic<std::int64_t> last_{0};
  std::atomic<std::int64_t> ema_{0};
  std::atomic<std::int64_t> min_{std::numeric_limits<std::int64_t>::max()};
  std::atomic<std::int64_t> max_{std::numeric_limits<std::int64_t>::min()};
};

struct StreamStatsSnapshot {
  std::uint64_t rx_total{0};
  std::uint64_t rx_stamp_missing{0};
  std::uint64_t rx_negative_delay{0};
  StatsMetricSnapshot delay_ns{};
  StatsMetricSnapshot interval_ns{};
  StatsMetricSnapshot interval_jitter_ns{};
};

class StreamStats final {
 public:
  void reset() noexcept {
    rx_total_.store(0, std::memory_order_relaxed);
    rx_stamp_missing_.store(0, std::memory_order_relaxed);
    rx_negative_delay_.store(0, std::memory_order_relaxed);
    last_rx_time_ns_.store(0, std::memory_order_relaxed);
    delay_ns_.reset();
    interval_ns_.reset();
    interval_jitter_ns_.reset();
  }

  void on_receive(std::int64_t recv_time_ns, std::int64_t stamp_ns,
                  std::uint32_t sample_every,
                  int ema_shift) noexcept {
    rx_total_.fetch_add(1, std::memory_order_relaxed);
    const bool sample = should_sample_(sample_every);

    const auto prev_rx = last_rx_time_ns_.exchange(recv_time_ns,
                                                   std::memory_order_relaxed);
    if (prev_rx > 0) {
      const auto interval = recv_time_ns - prev_rx;
      if (sample) {
        interval_ns_.add(interval, ema_shift);
        const auto interval_ema = interval_ns_.ema_relaxed();
        const auto jitter = (interval >= interval_ema) ? (interval - interval_ema)
                                                       : (interval_ema - interval);
        interval_jitter_ns_.add(jitter, ema_shift);
      }
    }

    if (stamp_ns <= 0) {
      rx_stamp_missing_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    const auto delay = recv_time_ns - stamp_ns;
    if (delay < 0) {
      rx_negative_delay_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    if (sample) {
      delay_ns_.add(delay, ema_shift);
    }
  }

  StreamStatsSnapshot snapshot() const noexcept {
    StreamStatsSnapshot out;
    out.rx_total = rx_total_.load(std::memory_order_relaxed);
    out.rx_stamp_missing = rx_stamp_missing_.load(std::memory_order_relaxed);
    out.rx_negative_delay = rx_negative_delay_.load(std::memory_order_relaxed);
    out.delay_ns = delay_ns_.snapshot();
    out.interval_ns = interval_ns_.snapshot();
    out.interval_jitter_ns = interval_jitter_ns_.snapshot();
    return out;
  }

 private:
  bool should_sample_(std::uint32_t sample_every) noexcept {
    if (sample_every <= 1)
      return true;
    const auto x =
        sample_gate_.fetch_add(1, std::memory_order_relaxed) + 1;
    return (x % sample_every) == 0;
  }

 private:
  std::atomic<std::uint64_t> rx_total_{0};
  std::atomic<std::uint64_t> rx_stamp_missing_{0};
  std::atomic<std::uint64_t> rx_negative_delay_{0};
  std::atomic<std::int64_t> last_rx_time_ns_{0};

  std::atomic<std::uint64_t> sample_gate_{0};
  StatsMetric delay_ns_{};
  StatsMetric interval_ns_{};
  StatsMetric interval_jitter_ns_{};
};

struct SyncStatsSnapshot {
  std::uint64_t tick_total{0};
  std::uint64_t tick_overrun{0};
  StatsMetricSnapshot wake_lateness_ns{};
  StatsMetricSnapshot compute_ns{};
  StatsMetricSnapshot age_arm_ns{};
  StatsMetricSnapshot age_leg_ns{};
  StatsMetricSnapshot age_imu_ns{};

  std::uint64_t missing_arm{0};
  std::uint64_t missing_leg{0};
  std::uint64_t missing_imu{0};

  std::uint64_t frame_written{0};
  std::uint64_t frame_complete{0};
  std::uint64_t frame_incomplete{0};
  std::uint64_t frame_aligned{0};
  std::uint64_t frame_unaligned_skew{0};
  std::uint64_t frame_incomplete_missing_arm{0};
  std::uint64_t frame_incomplete_missing_leg{0};
  std::uint64_t frame_incomplete_missing_imu{0};
};

class SyncStats final {
 public:
  void reset() noexcept {
    tick_total_.store(0, std::memory_order_relaxed);
    tick_overrun_.store(0, std::memory_order_relaxed);
    wake_lateness_ns_.reset();
    compute_ns_.reset();
    age_arm_ns_.reset();
    age_leg_ns_.reset();
    age_imu_ns_.reset();

    missing_arm_.store(0, std::memory_order_relaxed);
    missing_leg_.store(0, std::memory_order_relaxed);
    missing_imu_.store(0, std::memory_order_relaxed);

    frame_written_.store(0, std::memory_order_relaxed);
    frame_complete_.store(0, std::memory_order_relaxed);
    frame_incomplete_.store(0, std::memory_order_relaxed);
    frame_aligned_.store(0, std::memory_order_relaxed);
    frame_unaligned_skew_.store(0, std::memory_order_relaxed);
    frame_incomplete_missing_arm_.store(0, std::memory_order_relaxed);
    frame_incomplete_missing_leg_.store(0, std::memory_order_relaxed);
    frame_incomplete_missing_imu_.store(0, std::memory_order_relaxed);
  }

  void on_tick(std::int64_t wake_lateness_ns, std::int64_t compute_ns,
               bool overrun, bool arm_ok, bool leg_ok, bool imu_ok,
               std::int64_t age_arm_ns, std::int64_t age_leg_ns,
               std::int64_t age_imu_ns, std::uint32_t sample_every,
               int ema_shift) noexcept {
    tick_total_.fetch_add(1, std::memory_order_relaxed);
    if (overrun)
      tick_overrun_.fetch_add(1, std::memory_order_relaxed);
    if (should_sample_(sample_every)) {
      wake_lateness_ns_.add(wake_lateness_ns, ema_shift);
      compute_ns_.add(compute_ns, ema_shift);
      if (arm_ok)
        age_arm_ns_.add(age_arm_ns, ema_shift);
      if (leg_ok)
        age_leg_ns_.add(age_leg_ns, ema_shift);
      if (imu_ok)
        age_imu_ns_.add(age_imu_ns, ema_shift);
    }
  }

  void on_missing(bool arm_ok, bool leg_ok, bool imu_ok) noexcept {
    if (!arm_ok)
      missing_arm_.fetch_add(1, std::memory_order_relaxed);
    if (!leg_ok)
      missing_leg_.fetch_add(1, std::memory_order_relaxed);
    if (!imu_ok)
      missing_imu_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_frame_written(bool wrote) noexcept {
    if (wrote)
      frame_written_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_frame_flags(bool complete, bool aligned) noexcept {
    if (complete) {
      frame_complete_.fetch_add(1, std::memory_order_relaxed);
      if (aligned) {
        frame_aligned_.fetch_add(1, std::memory_order_relaxed);
      } else {
        frame_unaligned_skew_.fetch_add(1, std::memory_order_relaxed);
      }
    } else {
      frame_incomplete_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void on_incomplete_missing(bool arm_ok, bool leg_ok, bool imu_ok) noexcept {
    if (!arm_ok)
      frame_incomplete_missing_arm_.fetch_add(1, std::memory_order_relaxed);
    if (!leg_ok)
      frame_incomplete_missing_leg_.fetch_add(1, std::memory_order_relaxed);
    if (!imu_ok)
      frame_incomplete_missing_imu_.fetch_add(1, std::memory_order_relaxed);
  }

  SyncStatsSnapshot snapshot() const noexcept {
    SyncStatsSnapshot out;
    out.tick_total = tick_total_.load(std::memory_order_relaxed);
    out.tick_overrun = tick_overrun_.load(std::memory_order_relaxed);
    out.wake_lateness_ns = wake_lateness_ns_.snapshot();
    out.compute_ns = compute_ns_.snapshot();
    out.age_arm_ns = age_arm_ns_.snapshot();
    out.age_leg_ns = age_leg_ns_.snapshot();
    out.age_imu_ns = age_imu_ns_.snapshot();
    out.missing_arm = missing_arm_.load(std::memory_order_relaxed);
    out.missing_leg = missing_leg_.load(std::memory_order_relaxed);
    out.missing_imu = missing_imu_.load(std::memory_order_relaxed);
    out.frame_written = frame_written_.load(std::memory_order_relaxed);
    out.frame_complete = frame_complete_.load(std::memory_order_relaxed);
    out.frame_incomplete = frame_incomplete_.load(std::memory_order_relaxed);
    out.frame_aligned = frame_aligned_.load(std::memory_order_relaxed);
    out.frame_unaligned_skew =
        frame_unaligned_skew_.load(std::memory_order_relaxed);
    out.frame_incomplete_missing_arm =
        frame_incomplete_missing_arm_.load(std::memory_order_relaxed);
    out.frame_incomplete_missing_leg =
        frame_incomplete_missing_leg_.load(std::memory_order_relaxed);
    out.frame_incomplete_missing_imu =
        frame_incomplete_missing_imu_.load(std::memory_order_relaxed);
    return out;
  }

 private:
  bool should_sample_(std::uint32_t sample_every) noexcept {
    if (sample_every <= 1)
      return true;
    const auto x =
        sample_gate_.fetch_add(1, std::memory_order_relaxed) + 1;
    return (x % sample_every) == 0;
  }

 private:
  std::atomic<std::uint64_t> tick_total_{0};
  std::atomic<std::uint64_t> tick_overrun_{0};
  std::atomic<std::uint64_t> sample_gate_{0};
  StatsMetric wake_lateness_ns_{};
  StatsMetric compute_ns_{};
  StatsMetric age_arm_ns_{};
  StatsMetric age_leg_ns_{};
  StatsMetric age_imu_ns_{};

  std::atomic<std::uint64_t> missing_arm_{0};
  std::atomic<std::uint64_t> missing_leg_{0};
  std::atomic<std::uint64_t> missing_imu_{0};

  std::atomic<std::uint64_t> frame_written_{0};
  std::atomic<std::uint64_t> frame_complete_{0};
  std::atomic<std::uint64_t> frame_incomplete_{0};
  std::atomic<std::uint64_t> frame_aligned_{0};
  std::atomic<std::uint64_t> frame_unaligned_skew_{0};
  std::atomic<std::uint64_t> frame_incomplete_missing_arm_{0};
  std::atomic<std::uint64_t> frame_incomplete_missing_leg_{0};
  std::atomic<std::uint64_t> frame_incomplete_missing_imu_{0};
};

struct PublishStatsSnapshot {
  std::uint64_t attempts{0};
  std::uint64_t skipped_no_cmd{0};
  StatsMetricSnapshot duration_ns{};
};

class PublishStats final {
 public:
  void reset() noexcept {
    attempts_.store(0, std::memory_order_relaxed);
    skipped_no_cmd_.store(0, std::memory_order_relaxed);
    duration_ns_.reset();
  }

  void on_attempt(bool has_cmd) noexcept {
    attempts_.fetch_add(1, std::memory_order_relaxed);
    if (!has_cmd)
      skipped_no_cmd_.fetch_add(1, std::memory_order_relaxed);
  }

  void on_duration(std::int64_t duration_ns, std::uint32_t sample_every,
                   int ema_shift) noexcept {
    if (should_sample_(sample_every)) {
      duration_ns_.add(duration_ns, ema_shift);
    }
  }

  PublishStatsSnapshot snapshot() const noexcept {
    PublishStatsSnapshot out;
    out.attempts = attempts_.load(std::memory_order_relaxed);
    out.skipped_no_cmd = skipped_no_cmd_.load(std::memory_order_relaxed);
    out.duration_ns = duration_ns_.snapshot();
    return out;
  }

 private:
  bool should_sample_(std::uint32_t sample_every) noexcept {
    if (sample_every <= 1)
      return true;
    const auto x =
        sample_gate_.fetch_add(1, std::memory_order_relaxed) + 1;
    return (x % sample_every) == 0;
  }

 private:
  std::atomic<std::uint64_t> attempts_{0};
  std::atomic<std::uint64_t> skipped_no_cmd_{0};
  std::atomic<std::uint64_t> sample_gate_{0};
  StatsMetric duration_ns_{};
};

enum class WaitFrameStatus : std::uint8_t { Ok = 0, Timeout = 1, Stopped = 2 };

struct WaitFrameStatsSnapshot {
  std::uint64_t calls{0};
  std::uint64_t ok{0};
  std::uint64_t timeout{0};
  std::uint64_t stopped{0};
  StatsMetricSnapshot wait_ns{};
};

class WaitFrameStats final {
 public:
  void reset() noexcept {
    calls_.store(0, std::memory_order_relaxed);
    ok_.store(0, std::memory_order_relaxed);
    timeout_.store(0, std::memory_order_relaxed);
    stopped_.store(0, std::memory_order_relaxed);
    wait_ns_.reset();
  }

  void on_result(WaitFrameStatus st, std::int64_t wait_ns,
                 std::uint32_t sample_every, int ema_shift) noexcept {
    calls_.fetch_add(1, std::memory_order_relaxed);
    switch (st) {
      case WaitFrameStatus::Ok:
        ok_.fetch_add(1, std::memory_order_relaxed);
        break;
      case WaitFrameStatus::Timeout:
        timeout_.fetch_add(1, std::memory_order_relaxed);
        break;
      case WaitFrameStatus::Stopped:
        stopped_.fetch_add(1, std::memory_order_relaxed);
        break;
    }
    if (st == WaitFrameStatus::Ok && should_sample_(sample_every)) {
      wait_ns_.add(wait_ns, ema_shift);
    }
  }

  WaitFrameStatsSnapshot snapshot() const noexcept {
    WaitFrameStatsSnapshot out;
    out.calls = calls_.load(std::memory_order_relaxed);
    out.ok = ok_.load(std::memory_order_relaxed);
    out.timeout = timeout_.load(std::memory_order_relaxed);
    out.stopped = stopped_.load(std::memory_order_relaxed);
    out.wait_ns = wait_ns_.snapshot();
    return out;
  }

 private:
  bool should_sample_(std::uint32_t sample_every) noexcept {
    if (sample_every <= 1)
      return true;
    const auto x =
        sample_gate_.fetch_add(1, std::memory_order_relaxed) + 1;
    return (x % sample_every) == 0;
  }

 private:
  std::atomic<std::uint64_t> calls_{0};
  std::atomic<std::uint64_t> ok_{0};
  std::atomic<std::uint64_t> timeout_{0};
  std::atomic<std::uint64_t> stopped_{0};
  std::atomic<std::uint64_t> sample_gate_{0};
  StatsMetric wait_ns_{};
};

struct StatisticsSnapshot {
  bool enabled{false};
  std::uint32_t sample_every{1};
  int ema_shift{4};
  std::int64_t now_steady_ns{0};
  std::int64_t start_steady_ns{0};

  StreamStatsSnapshot arm_state{};
  StreamStatsSnapshot leg_state{};
  StreamStatsSnapshot imu{};

  PublishStatsSnapshot publish_arm{};
  PublishStatsSnapshot publish_leg{};
  PublishStatsSnapshot commit_total{};

  SyncStatsSnapshot sync{};
  WaitFrameStatsSnapshot wait_frame{};
};

class Statistics final {
 public:
  struct Config {
    bool enabled{false};
    std::uint32_t sample_every{1};
    int ema_shift{4};  // EMA alpha = 1 / (2^ema_shift), 0 means "last sample"
  };

  void reset(std::int64_t start_steady_ns) noexcept {
    start_steady_ns_.store(start_steady_ns, std::memory_order_relaxed);
    arm_state_.reset();
    leg_state_.reset();
    imu_.reset();
    publish_arm_.reset();
    publish_leg_.reset();
    commit_total_.reset();
    sync_.reset();
    wait_frame_.reset();
  }

  void set_config(Config cfg) noexcept {
    if (cfg.sample_every == 0)
      cfg.sample_every = 1;
    if (cfg.ema_shift < 0)
      cfg.ema_shift = 0;
    if (cfg.ema_shift > 30)
      cfg.ema_shift = 30;
    enabled_.store(cfg.enabled, std::memory_order_relaxed);
    sample_every_.store(cfg.sample_every, std::memory_order_relaxed);
    ema_shift_.store(cfg.ema_shift, std::memory_order_relaxed);
  }

  Config config() const noexcept {
    return Config{
        .enabled = enabled_.load(std::memory_order_relaxed),
        .sample_every = sample_every_.load(std::memory_order_relaxed),
        .ema_shift = ema_shift_.load(std::memory_order_relaxed),
    };
  }

  bool enabled() const noexcept {
    return enabled_.load(std::memory_order_relaxed);
  }

  void on_arm_state(std::int64_t recv_ns, std::int64_t stamp_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    arm_state_.on_receive(recv_ns, stamp_ns, cfg.sample_every, cfg.ema_shift);
  }

  void on_leg_state(std::int64_t recv_ns, std::int64_t stamp_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    leg_state_.on_receive(recv_ns, stamp_ns, cfg.sample_every, cfg.ema_shift);
  }

  void on_imu(std::int64_t recv_ns, std::int64_t stamp_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    imu_.on_receive(recv_ns, stamp_ns, cfg.sample_every, cfg.ema_shift);
  }

  void on_publish_arm_attempt(bool has_cmd) noexcept {
    if (!enabled())
      return;
    publish_arm_.on_attempt(has_cmd);
  }

  void on_publish_leg_attempt(bool has_cmd) noexcept {
    if (!enabled())
      return;
    publish_leg_.on_attempt(has_cmd);
  }

  void on_commit_attempt(bool has_cmd_any) noexcept {
    if (!enabled())
      return;
    commit_total_.on_attempt(has_cmd_any);
  }

  void on_publish_arm_duration(std::int64_t duration_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    publish_arm_.on_duration(duration_ns, cfg.sample_every, cfg.ema_shift);
  }

  void on_publish_leg_duration(std::int64_t duration_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    publish_leg_.on_duration(duration_ns, cfg.sample_every, cfg.ema_shift);
  }

  void on_commit_duration(std::int64_t duration_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    commit_total_.on_duration(duration_ns, cfg.sample_every, cfg.ema_shift);
  }

  void on_sync_tick(std::int64_t wake_lateness_ns, std::int64_t compute_ns,
                    bool overrun, bool arm_ok, bool leg_ok, bool imu_ok,
                    std::int64_t age_arm_ns, std::int64_t age_leg_ns,
                    std::int64_t age_imu_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    sync_.on_tick(wake_lateness_ns, compute_ns, overrun, arm_ok, leg_ok, imu_ok,
                  age_arm_ns, age_leg_ns, age_imu_ns, cfg.sample_every,
                  cfg.ema_shift);
  }

  void on_sync_missing(bool arm_ok, bool leg_ok, bool imu_ok) noexcept {
    if (!enabled())
      return;
    sync_.on_missing(arm_ok, leg_ok, imu_ok);
  }

  void on_sync_frame_written(bool wrote) noexcept {
    if (!enabled())
      return;
    sync_.on_frame_written(wrote);
  }

  void on_sync_frame_flags(bool complete, bool aligned) noexcept {
    if (!enabled())
      return;
    sync_.on_frame_flags(complete, aligned);
  }

  void on_sync_incomplete_missing(bool arm_ok, bool leg_ok,
                                  bool imu_ok) noexcept {
    if (!enabled())
      return;
    sync_.on_incomplete_missing(arm_ok, leg_ok, imu_ok);
  }

  void on_wait_frame(WaitFrameStatus st, std::int64_t wait_ns) noexcept {
    if (!enabled())
      return;
    const auto cfg = config_relaxed_();
    wait_frame_.on_result(st, wait_ns, cfg.sample_every, cfg.ema_shift);
  }

  StatisticsSnapshot snapshot(std::int64_t now_steady_ns) const noexcept {
    StatisticsSnapshot out;
    const auto cfg = config();
    out.enabled = cfg.enabled;
    out.sample_every = cfg.sample_every;
    out.ema_shift = cfg.ema_shift;
    out.now_steady_ns = now_steady_ns;
    out.start_steady_ns = start_steady_ns_.load(std::memory_order_relaxed);
    out.arm_state = arm_state_.snapshot();
    out.leg_state = leg_state_.snapshot();
    out.imu = imu_.snapshot();
    out.publish_arm = publish_arm_.snapshot();
    out.publish_leg = publish_leg_.snapshot();
    out.commit_total = commit_total_.snapshot();
    out.sync = sync_.snapshot();
    out.wait_frame = wait_frame_.snapshot();
    return out;
  }

 private:
  Config config_relaxed_() const noexcept {
    return Config{
        .enabled = enabled_.load(std::memory_order_relaxed),
        .sample_every = sample_every_.load(std::memory_order_relaxed),
        .ema_shift = ema_shift_.load(std::memory_order_relaxed),
    };
  }

 private:
  std::atomic<bool> enabled_{false};
  std::atomic<std::uint32_t> sample_every_{1};
  std::atomic<int> ema_shift_{4};
  std::atomic<std::int64_t> start_steady_ns_{0};

  StreamStats arm_state_{};
  StreamStats leg_state_{};
  StreamStats imu_{};

  PublishStats publish_arm_{};
  PublishStats publish_leg_{};
  PublishStats commit_total_{};

  SyncStats sync_{};
  WaitFrameStats wait_frame_{};
};

}  // namespace aimrl_sdk

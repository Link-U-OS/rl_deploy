#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

namespace aimrl_sdk {

template <class T> class RingBuffer {
public:
  explicit RingBuffer(std::uint32_t capacity)
      : capacity_(capacity), slots_(capacity), committed_(capacity) {
    for (auto &c : committed_)
      c.store(0, std::memory_order_relaxed);
  }

  std::uint32_t capacity() const noexcept { return capacity_; }

  template <class F> std::uint64_t write(F &&fill) {
    const auto idx =
        write_idx_.fetch_add(1, std::memory_order_relaxed) + 1; // start from 1
    const auto s = static_cast<std::uint32_t>(idx % capacity_);
    fill(slots_[s]);
    committed_[s].store(idx, std::memory_order_release);
    return idx;
  }

  bool read_at(std::uint64_t idx, T &out) const {
    const auto s = static_cast<std::uint32_t>(idx % capacity_);
    const auto c = committed_[s].load(std::memory_order_acquire);
    if (c != idx)
      return false;
    out = slots_[s];
    return true;
  }

  std::uint64_t latest_index() const noexcept {
    return write_idx_.load(std::memory_order_relaxed);
  }

private:
  std::uint32_t capacity_;
  std::vector<T> slots_;
  std::vector<std::atomic<std::uint64_t>> committed_;
  std::atomic<std::uint64_t> write_idx_{0};
};

} // namespace aimrl_sdk

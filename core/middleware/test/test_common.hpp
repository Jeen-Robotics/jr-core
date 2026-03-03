#pragma once

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

namespace jr::mw::test {

inline std::string unique_topic(const std::string& prefix) {
  static std::atomic<unsigned long long> counter{1};
  const auto id = counter.fetch_add(1, std::memory_order_relaxed);
  return prefix + "_" + std::to_string(id);
}

template <typename Fn>
inline bool wait_until(
  std::chrono::milliseconds timeout,
  std::chrono::milliseconds step,
  Fn&& condition
) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(step);
  }
  return condition();
}

} // namespace jr::mw::test

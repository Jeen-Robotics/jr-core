#pragma once

/// @file subscription_base.hpp
/// @brief Type-erased subscription base for runtime polymorphism

#include <atomic>

namespace jr::mw::detail {

/// Base class for type-erased subscriptions
class SubscriptionBase {
public:
  virtual ~SubscriptionBase() = default;

  /// Process one message if available, invoking the callback
  /// @return true if a message was processed
  /// @note Returns false immediately if cancelled
  virtual bool spin_once() = 0;

  /// Check if subscription is valid (underlying subscriber is functional)
  virtual bool valid() const = 0;

  /// Get file descriptor for epoll (Linux only, -1 otherwise)
  virtual int get_fd() const = 0;

  /// Mark subscription as cancelled (no more callbacks)
  void cancel() {
    cancelled_.store(true);
  }

  /// Check if cancelled
  bool is_cancelled() const {
    return cancelled_.load();
  }

protected:
  std::atomic<bool> cancelled_{false};
};

} // namespace jr::mw::detail

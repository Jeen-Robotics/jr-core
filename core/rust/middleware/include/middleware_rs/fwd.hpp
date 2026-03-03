#pragma once

/// @file fwd.hpp
/// @brief Forward declarations for middleware_rs

#include <cstdint>
#include <functional>
#include <memory>

namespace jr::mw {

/// QoS presets matching Rust middleware
enum class Qos : std::uint8_t {
  /// Reliable delivery with history buffer
  KeepLast = 0,
  /// Realtime: drop old messages, always get latest
  SensorData = 1,
};

// Opaque implementation types
namespace detail {
struct PublisherImpl;
struct SubscriberImpl;

/// Deleters for pimpl unique_ptrs (defined in wrapper.cpp)
void delete_publisher_impl(const PublisherImpl* p);
void delete_subscriber_impl(const SubscriberImpl* p);
} // namespace detail

/// Custom deleter for Publisher pimpl
struct PublisherDeleter {
  void operator()(const detail::PublisherImpl* p) const noexcept {
    detail::delete_publisher_impl(p);
  }
};

/// Custom deleter for Subscriber pimpl
struct SubscriberDeleter {
  void operator()(const detail::SubscriberImpl* p) const noexcept {
    detail::delete_subscriber_impl(p);
  }
};

// Unique pointers with custom deleters
using PublisherPtr = std::unique_ptr<detail::PublisherImpl, PublisherDeleter>;
using SubscriberPtr =
  std::unique_ptr<detail::SubscriberImpl, SubscriberDeleter>;

} // namespace jr::mw

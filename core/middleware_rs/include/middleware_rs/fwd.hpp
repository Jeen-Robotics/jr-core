#pragma once

/// @file fwd.hpp
/// @brief Forward declarations for middleware_rs

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace jr::mw::rust {

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
} // namespace detail

// Shared pointer deleters
using PublisherPtr = std::unique_ptr<detail::PublisherImpl>;
using SubscriberPtr = std::unique_ptr<detail::SubscriberImpl>;

} // namespace jr::mw::rust

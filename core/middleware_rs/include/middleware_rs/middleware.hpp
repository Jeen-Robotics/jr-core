#pragma once

/// @file middleware.hpp
/// @brief Rust-backed middleware C++ wrapper
///
/// High-level API that hides the FFI layer. Users interact with
/// typed publishers and subscribers using protobuf messages.
///
/// @example
/// ```cpp
/// #include <middleware_rs/middleware.hpp>
/// #include <my_msgs/sensor_data.pb.h>
///
/// int main() {
///     jr::mw::rust::init();
///
///     // Create typed publisher
///     auto pub = jr::mw::rust::advertise<my_msgs::SensorData>("sensor/data");
///     
///     // Create typed subscriber with callback
///     auto sub = jr::mw::rust::subscribe<my_msgs::SensorData>(
///         "sensor/data",
///         [](const my_msgs::SensorData& msg) {
///             std::cout << "Received: " << msg.value() << std::endl;
///         }
///     );
///     
///     // Publish
///     my_msgs::SensorData msg;
///     msg.set_value(42);
///     pub.publish(msg);
///     
///     // Poll for messages (drives callbacks)
///     while (running) {
///         sub.spin_once();
///     }
/// }
/// ```

#include <middleware_rs/fwd.hpp>
#include <middleware_rs/publisher.hpp>
#include <middleware_rs/subscriber.hpp>

namespace jr::mw::rust {

/// Initialize the Rust middleware (idempotent)
/// @return true on success
bool init();

/// Create a typed publisher
/// @tparam ProtoT Protobuf message type
/// @param topic Topic name
/// @param qos QoS preset (default: KeepLast)
/// @param capacity Buffer capacity for KeepLast (ignored for SensorData)
/// @return Publisher instance
template <typename ProtoT>
Publisher<ProtoT> advertise(
    const std::string& topic,
    Qos qos = Qos::KeepLast,
    std::size_t capacity = 16
) {
    auto impl = detail::create_publisher_impl(topic, qos, capacity);
    return Publisher<ProtoT>(topic, std::move(impl));
}

/// Create a typed subscriber with callback
/// @tparam ProtoT Protobuf message type
/// @param topic Topic name
/// @param callback Function to call on message receipt
/// @param qos QoS preset (default: KeepLast)
/// @param capacity Buffer capacity for KeepLast (ignored for SensorData)
/// @return Subscriber instance
template <typename ProtoT>
Subscriber<ProtoT> subscribe(
    const std::string& topic,
    std::function<void(const ProtoT&)> callback,
    Qos qos = Qos::KeepLast,
    std::size_t capacity = 16
) {
    auto impl = detail::create_subscriber_impl(topic, qos, capacity);
    return Subscriber<ProtoT>(topic, std::move(impl), std::move(callback));
}

/// Create a typed subscriber for polling (no callback)
/// @tparam ProtoT Protobuf message type
/// @param topic Topic name
/// @param qos QoS preset (default: KeepLast)
/// @param capacity Buffer capacity for KeepLast (ignored for SensorData)
/// @return Subscriber instance
template <typename ProtoT>
Subscriber<ProtoT> subscribe(
    const std::string& topic,
    Qos qos = Qos::KeepLast,
    std::size_t capacity = 16
) {
    auto impl = detail::create_subscriber_impl(topic, qos, capacity);
    return Subscriber<ProtoT>(topic, std::move(impl));
}

/// Check if a topic exists
bool topic_exists(const std::string& topic);

/// Get subscriber count for a topic
std::size_t subscriber_count(const std::string& topic);

} // namespace jr::mw::rust

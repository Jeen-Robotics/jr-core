#pragma once

/// @file middleware_impl.hpp
/// @brief Template implementation for Middleware

#include <middleware/detail/subscription_impl.hpp>
#include <middleware_rs/middleware.hpp>

namespace jr::mw {

template <typename ProtoT>
Subscription Middleware::do_subscribe_typed(
    const std::string& topic,
    const std::string& type_full_name,
    std::function<void(const ProtoT&)> callback,
    Qos qos,
    std::size_t capacity
) {
    // Check type consistency
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = topic_types_.find(topic);
        if (it != topic_types_.end() && it->second != type_full_name) {
            // Type mismatch - return invalid subscription
            return Subscription{};
        }
        topic_types_[topic] = type_full_name;
    }

    // Create Rust-backed subscriber (no callback - we poll manually)
    // Note: Qos enum is shared between middleware and middleware_rs
    auto sub = jr::mw::subscribe<ProtoT>(topic, qos, capacity);
    if (!sub.valid()) {
        return Subscription{};
    }

    // Allocate ID
    std::uint64_t id;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        id = next_id_++;
    }

    // Wrap in type-erased subscription (use shared_ptr for safe concurrent access)
    auto impl = std::make_shared<detail::SubscriptionImpl<ProtoT>>(
        std::move(sub),
        std::move(callback)
    );

    return make_subscription(id, std::move(impl));
}

} // namespace jr::mw

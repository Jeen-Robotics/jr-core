#pragma once

/// @file node.hpp
/// @brief ROS-like Node abstraction for pub/sub

#include <middleware/middleware.hpp>
#include <middleware/subscription.hpp>
#include <middleware_rs/middleware.hpp>  // For advertise<T>

#include <google/protobuf/message.h>

#include <functional>
#include <memory>
#include <string>

namespace jr::mw {

/// ROS-like Node abstraction
/// 
/// Provides a convenient interface for creating publishers and subscribers
/// within a named context.
///
/// @note Publishers use the global Rust middleware backend directly.
///       The per-node middleware instance (mw_) is used for subscriptions
///       and callback dispatch.
class Node {
public:
    virtual ~Node() = default;

    /// Create a node with the given name
    explicit Node(std::string node_name);
    
    /// Create a node with explicit middleware reference
    Node(std::string node_name, std::shared_ptr<Middleware> mw);

    /// Called periodically by spin()
    /// Override to implement periodic processing
    virtual void spin_once();

    /// Get node name
    const std::string& name() const noexcept;

    /// Create a typed publisher
    /// @note Publishers go through the global Rust backend, not the per-node mw_
    ///       The type is registered in mw_ for subscribe_any()/BagWriter support.
    template <typename ProtoT>
    Publisher<ProtoT> create_publisher(
        const std::string& topic,
        Qos qos = Qos::KeepLast,
        std::size_t capacity = 16
    ) {
        // Register type in C++ middleware for subscribe_any() / BagWriter support
        const auto* desc = ProtoT::descriptor();
        if (desc) {
            mw_->register_topic_type(topic, std::string(desc->full_name()));
        }
        return advertise<ProtoT>(topic, qos, capacity);
    }

    /// Create a typed subscription with callback
    template <typename ProtoT>
    Subscription create_subscription(
        const std::string& topic,
        std::function<void(const ProtoT&)> callback
    ) {
        return mw_->subscribe<ProtoT>(topic, std::move(callback));
    }

    /// Check if node is valid
    virtual bool valid() const noexcept;

private:
    std::string node_name_;
    std::shared_ptr<Middleware> mw_;
};

} // namespace jr::mw

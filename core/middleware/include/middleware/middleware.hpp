#pragma once

/// @file middleware.hpp
/// @brief ROS-like middleware API backed by Rust implementation
///
/// Provides a familiar Node-based API similar to ROS, with automatic
/// message dispatch via background threads.
///
/// @note The Rust backend uses a global singleton (OnceCell). All Middleware
///       instances share the same underlying transport. This means topics
///       are globally visible across all Middleware instances.

#include <middleware/subscription.hpp>
#include <middleware_rs/fwd.hpp>  // For Qos enum

#include <google/protobuf/message.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace jr::mw {

class Node;

namespace detail {
class SubscriptionBase;
} // namespace detail

/// Topic introspection info
struct TopicInfo {
    std::string name;
    std::string type_full_name;
};

/// Core middleware class
/// 
/// Manages pub/sub communication with automatic callback dispatch.
/// Uses Rust middleware backend for message routing.
///
/// @warning SINGLETON BACKEND: All Middleware instances share the same
///          underlying Rust transport. Topics are globally visible across
///          all instances. If you need topic isolation, use topic namespacing
///          (e.g., "/node1/topic" vs "/node2/topic").
///
/// @note Linux uses eventfd/poll for efficient message dispatch.
///       Non-Linux platforms use condition_variable with 10ms timeout.
class Middleware : public std::enable_shared_from_this<Middleware> {
public:
    ~Middleware();

    /// Stop the middleware and release resources
    void shutdown();

    /// Publish a protobuf message
    /// @param topic Topic name
    /// @param message Protobuf message to publish
    /// @param qos Quality of service for the internal publisher
    /// @param capacity Queue capacity for the internal publisher
    void publish(
        const std::string& topic,
        const google::protobuf::Message& message,
        Qos qos = Qos::KeepLast,
        std::size_t capacity = 16
    );

    /// Publish an already-serialized protobuf payload with explicit type name
    /// @param topic Topic name
    /// @param type_full_name Fully qualified protobuf type name
    /// @param payload Serialized protobuf data
    /// @param qos Quality of service for the internal publisher
    /// @param capacity Queue capacity for the internal publisher
    void publish_serialized(
        const std::string& topic,
        const std::string& type_full_name,
        const std::string& payload,
        Qos qos = Qos::KeepLast,
        std::size_t capacity = 16
    );

    /// Typed convenience publisher
    template <typename ProtoT>
    void publish(
        const std::string& topic,
        const ProtoT& message,
        Qos qos = Qos::KeepLast,
        std::size_t capacity = 16
    ) {
        static_assert(
            std::is_base_of_v<google::protobuf::Message, ProtoT>,
            "ProtoT must derive from google::protobuf::Message"
        );
        publish(topic, static_cast<const google::protobuf::Message&>(message), qos, capacity);
    }

    /// Subscribe with a concrete protobuf type
    template <typename ProtoT>
    Subscription subscribe(
        const std::string& topic,
        std::function<void(const ProtoT&)> callback,
        Qos qos = Qos::KeepLast,
        std::size_t capacity = 16
    ) {
        static_assert(
            std::is_base_of_v<google::protobuf::Message, ProtoT>,
            "ProtoT must derive from google::protobuf::Message"
        );
        
        const auto* desc = ProtoT::descriptor();
        std::string type_full_name;
        if (desc) {
            auto sv = desc->full_name();
            type_full_name.assign(sv.data(), sv.size());
        }
        
        return do_subscribe_typed<ProtoT>(topic, type_full_name, std::move(callback), qos, capacity);
    }

    /// Subscribe to all messages on a topic regardless of type
    /// Subscribe to all messages on a topic regardless of type.
    /// 
    /// Uses protobuf reflection to deserialize any message type. The type must
    /// be registered in the C++ topic_types_ registry before messages can be
    /// deserialized. Type registration happens when:
    /// - A typed publish() or publish_serialized() is called on the topic
    /// - A typed subscribe<T>() is called on the topic
    /// - Node::create_publisher<T>() is called for the topic
    /// 
    /// @warning Messages received before type registration are silently dropped.
    ///          For reliable recording, ensure at least one typed publisher or
    ///          subscriber exists on the topic before calling subscribe_any().
    /// 
    /// @param topic Topic name
    /// @param callback Callback receiving type name and deserialized message
    /// @param qos Quality of service
    /// @param capacity Queue capacity
    Subscription subscribe_any(
        const std::string& topic,
        std::function<void(const std::string&, const google::protobuf::Message&)> callback,
        Qos qos = Qos::KeepLast,
        std::size_t capacity = 16
    );

    /// Get topic names and types (C++ registry only, not Rust backend)
    std::vector<TopicInfo> get_topic_names_and_types() const;

    /// Register a topic type for subscribe_any() support
    /// Called by Node::create_publisher() to ensure type is available for BagWriter
    void register_topic_type(const std::string& topic, const std::string& type_full_name);

    /// Factory method
    static std::shared_ptr<Middleware> create();

private:
    friend class Subscription;
    friend class Node;

    Middleware();

    template <typename ProtoT>
    Subscription do_subscribe_typed(
        const std::string& topic,
        const std::string& type_full_name,
        std::function<void(const ProtoT&)> callback,
        Qos qos,
        std::size_t capacity
    );

    Subscription make_subscription(std::uint64_t id, std::shared_ptr<detail::SubscriptionBase> impl);
    void unregister_subscription(std::uint64_t id);
    bool subscription_valid(std::uint64_t id) const;
    
    void dispatcher_loop();
    void ensure_dispatcher();

    // mutable to allow locking in const methods (subscription_valid, get_topic_names_and_types)
    mutable std::mutex mutex_;
    // Use shared_ptr to allow safe concurrent access during dispatch
    std::unordered_map<std::uint64_t, std::shared_ptr<detail::SubscriptionBase>> subscriptions_;
    std::unordered_map<std::string, std::string> topic_types_;  // topic -> type_full_name
    std::unordered_map<std::string, std::shared_ptr<detail::PublisherImpl>> publishers_;
    std::uint64_t next_id_{1};
    
    std::thread dispatcher_;
    std::atomic<bool> shutdown_{false};
    std::atomic<bool> dispatcher_running_{false};
    bool init_failed_{false};  // Set if Rust backend initialization fails
    
    // Condition variable for non-Linux platforms to avoid busy-waiting
    std::condition_variable dispatch_cv_;
};

/// Get or create the global middleware instance
/// @note Also initializes the middleware if not already done
std::shared_ptr<Middleware> get();

/// Shutdown the global middleware instance
void shutdown();

/// Spin processing messages for a single node
/// @note spin() only exits when the global shutdown() is called.
///       Calling mw->shutdown() on a specific Middleware instance does NOT
///       cause spin() to exit. Use the global shutdown() for spin() loops.
void spin(const std::shared_ptr<Node>& node);

/// Spin processing messages for multiple nodes
/// @note spin() only exits when the global shutdown() is called.
///       Calling mw->shutdown() on a specific Middleware instance does NOT
///       cause spin() to exit. Use the global shutdown() for spin() loops.
void spin(const std::vector<std::shared_ptr<Node>>& nodes);

} // namespace jr::mw

// Include template implementation
#include <middleware/detail/middleware_impl.hpp>

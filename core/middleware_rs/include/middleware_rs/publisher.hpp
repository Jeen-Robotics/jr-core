#pragma once

/// @file publisher.hpp
/// @brief Typed publisher for Rust middleware

#include <middleware_rs/fwd.hpp>

#include <google/protobuf/message.h>

#include <string>
#include <type_traits>

namespace jr::mw::rust {

namespace detail {
/// Create publisher implementation (defined in wrapper.cpp)
PublisherPtr create_publisher_impl(const std::string& topic, Qos qos, std::size_t capacity);

/// Publish bytes (defined in wrapper.cpp)  
bool publish_impl(PublisherImpl* impl, const void* data, std::size_t len);

/// Check validity (defined in wrapper.cpp)
bool publisher_valid_impl(const PublisherImpl* impl);
} // namespace detail

/// Typed publisher for protobuf messages
/// @tparam ProtoT Protobuf message type (must derive from google::protobuf::Message)
template <typename ProtoT>
class Publisher {
    static_assert(
        std::is_base_of_v<google::protobuf::Message, ProtoT>,
        "ProtoT must derive from google::protobuf::Message"
    );

public:
    Publisher() = default;
    ~Publisher() = default;
    
    /// Move constructor
    Publisher(Publisher&&) noexcept = default;
    
    /// Move assignment
    Publisher& operator=(Publisher&&) noexcept = default;
    
    // Non-copyable
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

    /// Publish a message
    /// @param msg Protobuf message to publish
    /// @return true if published successfully (may have no receivers)
    bool publish(const ProtoT& msg) const {
        if (!valid()) {
            return false;
        }
        
        std::string buffer;
        if (!msg.SerializeToString(&buffer)) {
            return false;
        }
        
        return detail::publish_impl(impl_.get(), buffer.data(), buffer.size());
    }

    /// Check if publisher is valid
    bool valid() const noexcept {
        return impl_ && detail::publisher_valid_impl(impl_.get());
    }

    /// Boolean conversion for validity check
    explicit operator bool() const noexcept {
        return valid();
    }

    /// Get topic name
    const std::string& topic() const noexcept {
        return topic_;
    }

private:
    template <typename T>
    friend Publisher<T> advertise(const std::string&, Qos, std::size_t);

    Publisher(std::string topic, PublisherPtr impl)
        : topic_(std::move(topic))
        , impl_(std::move(impl))
    {}

    std::string topic_;
    PublisherPtr impl_;
};

} // namespace jr::mw::rust

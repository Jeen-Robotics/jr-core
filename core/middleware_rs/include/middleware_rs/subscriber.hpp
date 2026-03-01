#pragma once

/// @file subscriber.hpp
/// @brief Typed subscriber for Rust middleware

#include <middleware_rs/fwd.hpp>

#ifdef __linux__
#include <poll.h>
#endif

#include <google/protobuf/message.h>

#include <chrono>
#include <optional>
#include <thread>
#include <vector>

namespace jr::mw::rust {

namespace detail {
/// Create subscriber implementation (defined in wrapper.cpp)
SubscriberPtr create_subscriber_impl(const std::string& topic, Qos qos, std::size_t capacity);

/// Check validity (defined in wrapper.cpp)
bool subscriber_valid_impl(const SubscriberImpl* impl);

/// Get eventfd (defined in wrapper.cpp)
int subscriber_fd_impl(const SubscriberImpl* impl);

/// Try receive (defined in wrapper.cpp)
/// @return {has_message, data, closed}
struct RecvResult {
    bool has_message;
    std::vector<std::uint8_t> data;
    bool closed;
};
RecvResult subscriber_recv_impl(SubscriberImpl* impl);
} // namespace detail

/// Typed subscriber for protobuf messages
/// @tparam ProtoT Protobuf message type (must derive from google::protobuf::Message)
template <typename ProtoT>
class Subscriber {
    static_assert(
        std::is_base_of_v<google::protobuf::Message, ProtoT>,
        "ProtoT must derive from google::protobuf::Message"
    );

public:
    using Callback = std::function<void(const ProtoT&)>;

    Subscriber() = default;
    ~Subscriber() = default;
    
    /// Move constructor
    Subscriber(Subscriber&&) noexcept = default;
    
    /// Move assignment
    Subscriber& operator=(Subscriber&&) noexcept = default;
    
    // Non-copyable
    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;

    /// Try to receive a message (non-blocking)
    /// @return Message if available, nullopt otherwise
    std::optional<ProtoT> try_recv() {
        if (!valid()) {
            return std::nullopt;
        }
        
        auto result = detail::subscriber_recv_impl(impl_.get());
        closed_ = result.closed;
        
        if (!result.has_message) {
            return std::nullopt;
        }
        
        ProtoT msg;
        if (!msg.ParseFromArray(result.data.data(), static_cast<int>(result.data.size()))) {
            return std::nullopt;  // Parse failed
        }
        
        return msg;
    }

    /// Process one message if available, invoking the callback
    /// @return true if a message was processed
    bool spin_once() {
        auto msg = try_recv();
        if (msg && callback_) {
            callback_(*msg);
            return true;
        }
        return false;
    }

    /// Process messages for a duration, invoking callbacks
    /// @param timeout Maximum time to spin
    /// @return Number of messages processed
    std::size_t spin_for(std::chrono::milliseconds timeout) {
        std::size_t count = 0;
        auto deadline = std::chrono::steady_clock::now() + timeout;
        
        while (std::chrono::steady_clock::now() < deadline) {
            if (spin_once()) {
                ++count;
            } else {
                // Brief sleep to avoid busy-waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
        return count;
    }

    /// Wait for a message using poll (Linux) or sleep (other platforms)
    /// @param timeout Maximum time to wait (-1 for infinite)
    /// @return true if a message is likely available
    bool wait(std::chrono::milliseconds timeout = std::chrono::milliseconds(-1)) const {
#ifdef __linux__
        int fd = detail::subscriber_fd_impl(impl_.get());
        if (fd < 0) {
            return false;
        }
        
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        
        int ret = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
        return ret > 0 && (pfd.revents & POLLIN);
#else
        // On non-Linux, fall back to sleep
        std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(10)));
        return true;  // Caller should try_recv anyway
#endif
    }

    /// Get the eventfd for custom epoll integration
    /// @return File descriptor, or -1 if not available
    int get_fd() const noexcept {
        if (!impl_) return -1;
        return detail::subscriber_fd_impl(impl_.get());
    }

    /// Check if subscriber is valid
    bool valid() const noexcept {
        return impl_ && detail::subscriber_valid_impl(impl_.get());
    }

    /// Boolean conversion for validity check
    explicit operator bool() const noexcept {
        return valid();
    }

    /// Check if the channel is closed
    bool closed() const noexcept {
        return closed_;
    }

    /// Get topic name
    const std::string& topic() const noexcept {
        return topic_;
    }

    /// Set callback for spin_once/spin_for
    void set_callback(Callback cb) {
        callback_ = std::move(cb);
    }

private:
    template <typename T>
    friend Subscriber<T> subscribe(const std::string&, std::function<void(const T&)>, Qos, std::size_t);
    
    template <typename T>
    friend Subscriber<T> subscribe(const std::string&, Qos, std::size_t);

    Subscriber(std::string topic, SubscriberPtr impl, Callback callback = nullptr)
        : topic_(std::move(topic))
        , impl_(std::move(impl))
        , callback_(std::move(callback))
    {}

    std::string topic_;
    SubscriberPtr impl_;
    Callback callback_;
    bool closed_ = false;
};

} // namespace jr::mw::rust

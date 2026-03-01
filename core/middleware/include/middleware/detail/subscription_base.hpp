#pragma once

/// @file subscription_base.hpp
/// @brief Type-erased subscription base for runtime polymorphism

namespace jr::mw::detail {

/// Base class for type-erased subscriptions
class SubscriptionBase {
public:
    virtual ~SubscriptionBase() = default;
    
    /// Process one message if available, invoking the callback
    /// @return true if a message was processed
    virtual bool spin_once() = 0;
    
    /// Check if subscription is valid
    virtual bool valid() const = 0;
    
    /// Get file descriptor for epoll (Linux only, -1 otherwise)
    virtual int get_fd() const = 0;
};

} // namespace jr::mw::detail

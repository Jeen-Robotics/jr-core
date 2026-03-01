#pragma once

/// @file subscription_impl.hpp
/// @brief Typed subscription implementation wrapping Rust subscriber

#include <middleware/detail/subscription_base.hpp>
#include <middleware_rs/subscriber.hpp>

#include <google/protobuf/message.h>

#include <functional>
#include <type_traits>

namespace jr::mw::detail {

/// Typed subscription implementation
/// @tparam ProtoT Protobuf message type
template <typename ProtoT>
class SubscriptionImpl final : public SubscriptionBase {
    static_assert(
        std::is_base_of_v<google::protobuf::Message, ProtoT>,
        "ProtoT must derive from google::protobuf::Message"
    );

public:
    using Callback = std::function<void(const ProtoT&)>;

    SubscriptionImpl(jr::mw::Subscriber<ProtoT> sub, Callback callback)
        : sub_(std::move(sub))
        , callback_(std::move(callback))
    {}

    bool spin_once() override {
        auto msg = sub_.try_recv();
        if (msg && callback_) {
            callback_(*msg);
            return true;
        }
        return false;
    }

    bool valid() const override {
        return sub_.valid();
    }

    int get_fd() const override {
        return sub_.get_fd();
    }

private:
    jr::mw::Subscriber<ProtoT> sub_;
    Callback callback_;
};

} // namespace jr::mw::detail

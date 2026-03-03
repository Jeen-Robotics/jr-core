#pragma once

/// @file subscription_impl.hpp
/// @brief Typed subscription implementation wrapping transport subscriber

#include <middleware/detail/subscription_base.hpp>
#include <middleware/detail/transport.hpp>

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

  SubscriptionImpl(
    std::shared_ptr<SubscriberHandle> sub,
    Callback callback,
    Qos qos
  )
      : sub_(std::move(sub))
      , callback_(std::move(callback))
      , qos_(qos) {
  }

  bool spin_once() override {
    // Check if cancelled before processing
    if (is_cancelled()) {
      return false;
    }

    auto result = subscriber_recv(sub_);
    if (!result.has_message) {
      return false;
    }

    // SensorData semantics: collapse buffered samples to latest before callback.
    if (qos_ == Qos::SensorData) {
      while (true) {
        auto latest = subscriber_recv(sub_);
        if (!latest.has_message) {
          break;
        }
        result = std::move(latest);
      }
    }

    ProtoT msg;
    if (!msg.ParseFromArray(
          result.data.data(),
          static_cast<int>(result.data.size())
        )) {
      return false;
    }

    if (callback_) {
      callback_(msg);
      return true;
    }
    return false;
  }

  bool valid() const override {
    return subscriber_valid(sub_);
  }

  int get_fd() const override {
    return subscriber_fd(sub_);
  }

private:
  std::shared_ptr<SubscriberHandle> sub_;
  Callback callback_;
  Qos qos_{Qos::KeepLast};
};

} // namespace jr::mw::detail

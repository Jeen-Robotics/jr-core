#pragma once

/// @file subscription_any_impl.hpp
/// @brief Dynamic subscription implementation using protobuf reflection

#include <middleware/detail/subscription_base.hpp>
#include <middleware/detail/transport.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace jr::mw::detail {

/// Subscription that receives raw bytes and deserializes using protobuf
/// reflection. Requires access to the topic_types registry to look up message
/// types.
class SubscriptionAnyImpl final : public SubscriptionBase {
public:
  using Callback =
    std::function<void(const std::string&, const google::protobuf::Message&)>;
  using TypeRegistry = std::unordered_map<std::string, std::string>;

  SubscriptionAnyImpl(
    std::shared_ptr<SubscriberHandle> impl,
    std::string topic,
    Callback callback,
    std::mutex& registry_mutex,
    const TypeRegistry& topic_types
  )
      : impl_(std::move(impl))
      , topic_(std::move(topic))
      , callback_(std::move(callback))
      , registry_mutex_(registry_mutex)
      , topic_types_(topic_types)
      , pool_(google::protobuf::DescriptorPool::generated_pool())
      , factory_(google::protobuf::MessageFactory::generated_factory()) {
  }

  bool spin_once() override {
    if (is_cancelled()) {
      return false;
    }

    if (!subscriber_valid(impl_)) {
      return false;
    }

    auto result = subscriber_recv(impl_);
    if (!result.has_message) {
      return false;
    }

    // Look up the type from the registry
    std::string type_full_name;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      auto it = topic_types_.find(topic_);
      if (it != topic_types_.end()) {
        type_full_name = it->second;
      }
    }

    if (type_full_name.empty()) {
      // Type not registered yet - skip this message
      return false;
    }

    // Get or cache the descriptor
    if (type_full_name != cached_type_name_) {
      cached_type_name_ = type_full_name;
      cached_descriptor_ =
        pool_ ? pool_->FindMessageTypeByName(type_full_name) : nullptr;
    }

    // Deserialize the message
    if (cached_descriptor_ && factory_ && callback_) {
      const auto* prototype = factory_->GetPrototype(cached_descriptor_);
      if (prototype) {
        std::unique_ptr<google::protobuf::Message> msg(prototype->New());
        if (msg && msg->ParseFromArray(
                     result.data.data(),
                     static_cast<int>(result.data.size())
                   )) {
          callback_(type_full_name, *msg);
          return true;
        }
      }
    }

    return false;
  }

  bool valid() const override {
    return subscriber_valid(impl_);
  }

  int get_fd() const override {
    return subscriber_fd(impl_);
  }

private:
  std::shared_ptr<SubscriberHandle> impl_;
  std::string topic_;
  Callback callback_;
  std::mutex& registry_mutex_;
  const TypeRegistry& topic_types_;
  const google::protobuf::DescriptorPool* pool_;
  google::protobuf::MessageFactory* factory_;

  // Cached descriptor (type can change theoretically, but usually doesn't)
  std::string cached_type_name_;
  const google::protobuf::Descriptor* cached_descriptor_{nullptr};
};

} // namespace jr::mw::detail

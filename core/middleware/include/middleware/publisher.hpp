#pragma once

/// @file publisher.hpp
/// @brief Typed publisher for protobuf messages

#include <google/protobuf/message_lite.h>

#include <functional>
#include <string>
#include <type_traits>
#include <utility>

namespace jr::mw {

template <typename ProtoT>
class Publisher {
  static_assert(
    std::is_base_of_v<google::protobuf::MessageLite, ProtoT>,
    "ProtoT must derive from google::protobuf::MessageLite"
  );

public:
  Publisher() = default;
  ~Publisher() = default;

  Publisher(Publisher&&) noexcept = default;
  Publisher& operator=(Publisher&&) noexcept = default;

  Publisher(const Publisher&) = delete;
  Publisher& operator=(const Publisher&) = delete;

  bool publish(const ProtoT& msg) const {
    if (!publish_fn_) {
      return false;
    }
    return publish_fn_(msg);
  }

  bool valid() const noexcept {
    return static_cast<bool>(publish_fn_);
  }

  explicit operator bool() const noexcept {
    return valid();
  }

  const std::string& topic() const noexcept {
    return topic_;
  }

private:
  friend class Node;

  Publisher(std::string topic, std::function<bool(const ProtoT&)> publish_fn)
      : topic_(std::move(topic))
      , publish_fn_(std::move(publish_fn)) {
  }

  std::string topic_;
  std::function<bool(const ProtoT&)> publish_fn_;
};

} // namespace jr::mw

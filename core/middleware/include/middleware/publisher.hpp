#pragma once

#include <middleware/middleware.hpp>

#include <memory>
#include <string>

namespace jr::mw {

template <typename ProtoT>
class Publisher {
public:
  Publisher() = default;

  explicit Publisher(std::string topic)
      : topic_(std::move(topic))
      , owner_(get()) {
  }

  Publisher(std::string topic, std::weak_ptr<Middleware> owner)
      : topic_(std::move(topic))
      , owner_(std::move(owner)) {
  }

  void publish(const ProtoT& message) const {
    if (auto owner = owner_.lock()) {
      owner->publish(topic_, message);
    }
  }

  bool valid() const noexcept {
    return !topic_.empty() && !owner_.expired();
  }

private:
  std::string topic_;
  std::weak_ptr<Middleware> owner_;
};

} // namespace jr::mw
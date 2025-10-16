#include "middleware/subscription.hpp"

#include <middleware/middleware.hpp>

namespace jr::mw {

Subscription::Subscription(Subscription&& other) noexcept
    : owner_(std::move(other.owner_))
    , id_(other.id_) {
  other.id_ = 0;
}

Subscription& Subscription::operator=(Subscription&& other) noexcept {
  if (this != &other) {
    unsubscribe();
    owner_ = std::move(other.owner_);
    id_ = other.id_;
    other.id_ = 0;
  }
  return *this;
}

Subscription::~Subscription() {
  unsubscribe();
}

void Subscription::unsubscribe() {
  if (id_ == 0) {
    return;
  }

  if (const auto owner = owner_.lock()) {
    owner->do_unsubscribe(id_);
  }

  id_ = 0;
}

bool Subscription::valid() const noexcept {
  return id_ != 0;
}

Subscription::Subscription(
  std::weak_ptr<Middleware> owner,
  const std::uint64_t id
)
    : owner_(std::move(owner))
    , id_(id) {
}

} // namespace jr::mw
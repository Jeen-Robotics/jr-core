#pragma once

#include <cstdint>
#include <memory>

namespace jr::mw {

class Middleware;

class Subscription {
public:
  Subscription() = default;
  Subscription(const Subscription&) = delete;
  Subscription& operator=(const Subscription&) = delete;

  Subscription(Subscription&& other) noexcept;
  Subscription& operator=(Subscription&& other) noexcept;

  ~Subscription();

  void unsubscribe();

  bool valid() const noexcept;

private:
  friend class Middleware;

  Subscription(std::weak_ptr<Middleware> owner, std::uint64_t id);

  std::weak_ptr<Middleware> owner_;
  std::uint64_t id_{0};
};

} // namespace jr::mw
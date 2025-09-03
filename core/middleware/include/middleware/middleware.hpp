#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <utility>

namespace jr::mw {

enum class BackendKind : int {
  InProcess = 0,
};

class Middleware;

class Subscription {
public:
  Subscription() = default;
  Subscription(const Subscription&) = delete;
  Subscription& operator=(const Subscription&) = delete;

  Subscription(Subscription&& other) noexcept
      : owner_(std::move(other.owner_))
      , id_(other.id_) {
    other.id_ = 0;
  }

  Subscription& operator=(Subscription&& other) noexcept {
    if (this != &other) {
      unsubscribe();
      owner_ = std::move(other.owner_);
      id_ = other.id_;
      other.id_ = 0;
    }
    return *this;
  }

  ~Subscription() {
    unsubscribe();
  }

  void unsubscribe();

  bool valid() const noexcept {
    return id_ != 0;
  }

private:
  friend class Middleware;
  Subscription(std::weak_ptr<Middleware> owner, std::uint64_t id)
      : owner_(std::move(owner))
      , id_(id) {
  }

  std::weak_ptr<Middleware> owner_;
  std::uint64_t id_{0};
};

class Middleware : public std::enable_shared_from_this<Middleware> {
public:
  virtual ~Middleware() = default;

  template <typename MessageT>
  void publish(const std::string& topic, const MessageT& message) {
    auto payload = std::make_shared<MessageT>(message);
    do_publish(topic, std::type_index(typeid(MessageT)), std::move(payload));
  }

  template <typename MessageT>
  Subscription subscribe(
    const std::string& topic,
    std::function<void(const MessageT&)> callback
  ) {
    auto adapter = [cb = std::move(callback)](std::shared_ptr<void> any_ptr) {
      const MessageT& ref = *std::static_pointer_cast<MessageT>(any_ptr);
      cb(ref);
    };
    return do_subscribe(
      topic,
      std::type_index(typeid(MessageT)),
      std::move(adapter)
    );
  }

  static std::shared_ptr<Middleware> create(
    BackendKind backend = BackendKind::InProcess
  );

protected:
  friend class Subscription;

  Subscription make_subscription(std::uint64_t id) {
    return Subscription{weak_from_this(), id};
  }

  virtual Subscription do_subscribe(
    const std::string& topic,
    std::type_index type,
    std::function<void(std::shared_ptr<void>)> callback
  ) = 0;

  virtual void do_publish(
    const std::string& topic,
    std::type_index type,
    std::shared_ptr<void> payload
  ) = 0;

  virtual void do_unsubscribe(std::uint64_t id) = 0;
};

inline void Subscription::unsubscribe() {
  if (id_ == 0)
    return;
  if (auto owner = owner_.lock()) {
    owner->do_unsubscribe(id_);
  }
  id_ = 0;
}

std::shared_ptr<Middleware> get(BackendKind backend = BackendKind::InProcess);

} // namespace jr::mw
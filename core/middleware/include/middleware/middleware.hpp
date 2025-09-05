#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

namespace jr::mw {

enum class BackendKind : int {
  InProcess = 0,
};

class Middleware;
template <typename MessageT>
class Publisher;
class Node;

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

  // Stop the middleware background processing and release resources
  virtual void shutdown() = 0;

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

template <typename MessageT>
class Publisher {
public:
  Publisher(std::string topic, std::weak_ptr<Middleware> owner)
      : topic_(std::move(topic))
      , owner_(std::move(owner)) {
  }

  void publish(const MessageT& message) const {
    if (auto owner = owner_.lock()) {
      owner->template publish<MessageT>(topic_, message);
    }
  }

  bool valid() const noexcept {
    return !topic_.empty() && !owner_.expired();
  }

private:
  std::string topic_;
  std::weak_ptr<Middleware> owner_;
};

class Node {
public:
  virtual ~Node() = default;
  explicit Node(std::string node_name, std::shared_ptr<Middleware> mw)
      : node_name_(std::move(node_name))
      , mw_(std::move(mw)) {
  }

  explicit Node(std::string node_name)
      : Node(std::move(node_name), get()) {
  }

  const std::string& name() const noexcept {
    return node_name_;
  }

  template <typename MessageT>
  Publisher<MessageT> create_publisher(const std::string& topic) {
    return Publisher<MessageT>{topic, mw_};
  }

  template <typename MessageT>
  Subscription create_subscription(
    const std::string& topic,
    std::function<void(const MessageT&)> callback
  ) {
    return mw_->subscribe<MessageT>(topic, std::move(callback));
  }

  bool valid() const noexcept {
    return static_cast<bool>(mw_);
  }

private:
  std::string node_name_{};
  std::shared_ptr<Middleware> mw_{};
};

void init(BackendKind backend = BackendKind::InProcess);
void spin(std::shared_ptr<Node> node);
void spin(const std::vector<std::shared_ptr<Node>>& nodes);
void shutdown();

} // namespace jr::mw
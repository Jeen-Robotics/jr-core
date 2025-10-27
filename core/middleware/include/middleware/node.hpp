#pragma once

#include <middleware/middleware.hpp>
#include <middleware/publisher.hpp>
#include <middleware/subscription.hpp>

namespace jr::mw {

class Node {
public:
  virtual ~Node() = default;

  explicit Node(std::string node_name);
  explicit Node(std::string node_name, std::shared_ptr<Middleware> mw);

  virtual void spin_once();

  const std::string& name() const noexcept;

  template <typename ProtoT>
  Publisher<ProtoT> create_publisher(const std::string& topic) {
    return Publisher<ProtoT>{topic, mw_};
  }

  template <typename ProtoT>
  Subscription create_subscription(
    const std::string& topic,
    std::function<void(const ProtoT&)> callback
  ) {
    return mw_->subscribe<ProtoT>(topic, std::move(callback));
  }

  Subscription create_dynamic_subscription(
    const std::string& topic,
    const std::string& type_full_name,
    std::function<void(const google::protobuf::Message&)> callback
  ) const;

  Subscription create_subscription_any(
    const std::string& topic,
    std::function<void(const std::string&, const google::protobuf::Message&)>
      callback
  ) const;

  virtual bool valid() const noexcept;

private:
  std::string node_name_{};
  std::shared_ptr<Middleware> mw_{};
};

} // namespace jr::mw
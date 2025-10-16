#include "middleware/node.hpp"

namespace jr::mw {

Node::Node(std::string node_name)
    : Node(std::move(node_name), get()) {
}

Node::Node(std::string node_name, std::shared_ptr<Middleware> mw)
    : node_name_(std::move(node_name))
    , mw_(std::move(mw)) {
}

void Node::spin_once() {
  // NOTE: child classes should override this method
}

const std::string& Node::name() const noexcept {
  return node_name_;
}

Subscription Node::create_dynamic_subscription(
  const std::string& topic,
  const std::string& type_full_name,
  std::function<void(const google::protobuf::Message&)> callback
) const {
  return mw_->subscribe(topic, type_full_name, std::move(callback));
}

Subscription Node::create_subscription_any(
  const std::string& topic,
  std::function<void(const std::string&, const google::protobuf::Message&)>
    callback
) const {
  return mw_->subscribe_any(topic, std::move(callback));
}

bool Node::valid() const noexcept {
  return static_cast<bool>(mw_);
}

} // namespace jr::mw
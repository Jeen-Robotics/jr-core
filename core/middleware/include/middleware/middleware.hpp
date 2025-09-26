#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <google/protobuf/message.h>

#include <middleware/subscription.hpp>

namespace jr::mw {

class Node;

enum class BackendKind : int {
  InProcess = 0,
};

struct TopicInfo {
  std::string name;
  std::string type_full_name; // empty if unknown yet
};

class Middleware : public std::enable_shared_from_this<Middleware> {
public:
  virtual ~Middleware() = default;

  // Stop the middleware background processing and release resources
  virtual void shutdown() = 0;

  // Publish a protobuf message
  void publish(
    const std::string& topic,
    const google::protobuf::Message& message
  );

  // Publish an already-serialized protobuf payload with explicit type name
  void publish_serialized(
    const std::string& topic,
    const std::string& type_full_name,
    const std::string& payload
  );

  // Typed convenience publisher
  template <typename ProtoT>
  void publish(const std::string& topic, const ProtoT& message) {
    static_assert(
      std::is_base_of<google::protobuf::Message, ProtoT>::value,
      "ProtoT must derive from google::protobuf::Message"
    );
    publish(topic, static_cast<const google::protobuf::Message&>(message));
  }

  // Subscribe with a concrete protobuf type
  template <typename ProtoT>
  Subscription subscribe(
    const std::string& topic,
    std::function<void(const ProtoT&)> callback
  ) {
    static_assert(
      std::is_base_of<google::protobuf::Message, ProtoT>::value,
      "ProtoT must derive from google::protobuf::Message"
    );
    const auto* desc = ProtoT::descriptor();
    std::string type_full_name;
    if (desc) {
      auto sv = desc->full_name();
      type_full_name.assign(sv.data(), sv.size());
    }
    return do_subscribe_typed(
      topic,
      type_full_name,
      [cb = std::move(callback)](const google::protobuf::Message& any_msg) {
        const auto* typed = dynamic_cast<const ProtoT*>(&any_msg);
        if (typed)
          cb(*typed);
      }
    );
  }

  // Subscribe dynamically by type name (e.g. "jr.test.Int32")
  Subscription subscribe(
    const std::string& topic,
    const std::string& type_full_name,
    std::function<void(const google::protobuf::Message&)> callback
  );

  // Subscribe to all messages on a topic regardless of type
  Subscription subscribe_any(
    const std::string& topic,
    std::function<
      void(const std::string& /*type_full_name*/, const google::protobuf::Message&)>
      callback
  );

  // Introspection similar to ROS: get topic names and types
  virtual std::vector<TopicInfo> get_topic_names_and_types() const = 0;

  static std::shared_ptr<Middleware> create(
    BackendKind backend = BackendKind::InProcess
  );

protected:
  friend class Subscription;

  Subscription make_subscription(std::uint64_t id) {
    return Subscription{weak_from_this(), id};
  }

  // Typed subscribe implementation entry (type string + base Message callback)
  virtual Subscription do_subscribe_typed(
    const std::string& topic,
    const std::string& type_full_name,
    std::function<void(const google::protobuf::Message&)> callback
  ) = 0;

  // Dynamic subscribe-any entry
  virtual Subscription do_subscribe_any(
    const std::string& topic,
    std::function<void(const std::string&, const google::protobuf::Message&)>
      callback
  ) = 0;

  // Publish serialized payload
  virtual void do_publish_serialized(
    const std::string& topic,
    const std::string& type_full_name,
    std::string payload
  ) = 0;

  virtual void do_unsubscribe(std::uint64_t id) = 0;
};

std::shared_ptr<Middleware> get(BackendKind backend = BackendKind::InProcess);

void init(BackendKind backend = BackendKind::InProcess);
void spin(std::shared_ptr<Node> node);
void spin(const std::vector<std::shared_ptr<Node>>& nodes);
void shutdown();

} // namespace jr::mw
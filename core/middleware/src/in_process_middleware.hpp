#pragma once

#include <middleware/middleware.hpp>

#include <deque>
#include <thread>
#include <unordered_map>

namespace jr::mw {

struct SubscriptionRecordTyped {
  std::uint64_t id;
  std::string type_full_name;
  std::function<void(const google::protobuf::Message&)> callback;
};

struct SubscriptionRecordAny {
  std::uint64_t id;
  std::function<void(const std::string&, const google::protobuf::Message&)>
    callback;
};

class InProcessMiddleware final : public Middleware {
public:
  InProcessMiddleware();
  ~InProcessMiddleware() override;

  Subscription do_subscribe_typed(
    const std::string& topic,
    const std::string& type_full_name,
    std::function<void(const google::protobuf::Message&)> callback
  ) override;

  Subscription do_subscribe_any(
    const std::string& topic,
    std::function<void(const std::string&, const google::protobuf::Message&)>
      callback
  ) override;

  void do_publish_serialized(
    const std::string& topic,
    const std::string& type_full_name,
    std::string payload
  ) override;

  void do_unsubscribe(std::uint64_t id) override;

  void shutdown() override;

  std::vector<TopicInfo> get_topic_names_and_types() const override;

private:
  struct EncodedMessage {
    std::string type_full_name;
    std::string payload;
    std::uint64_t timestamp_ns{0};
  };

  struct TopicState {
    std::deque<EncodedMessage> queue;
    std::vector<SubscriptionRecordTyped> subscriptions_typed;
    std::vector<SubscriptionRecordAny> subscriptions_any;
    std::condition_variable cv;
    std::mutex topic_mutex; // not used; using global mutex for simplicity
    std::thread dispatcher;
    bool is_active{false};
    std::string type_full_name;
    bool has_type{false};
  };

  static std::uint64_t now_ns();

  static std::unique_ptr<google::protobuf::Message> decode_message(
    const std::string& type_full_name,
    const std::string& payload
  );

  void ensure_dispatcher(const std::string& topic, TopicState& state);

  void dispatch_loop(const std::string& topic);

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicState> topics_;
  std::unordered_map<std::uint64_t, std::string> subscription_to_topic_;
  std::uint64_t next_subscription_id_;
  bool is_shutdown_;
  static constexpr std::size_t max_queue_size_ = 1; // keep-latest policy
};

} // namespace jr::mw
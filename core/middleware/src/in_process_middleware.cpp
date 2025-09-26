#include "in_process_middleware.hpp"

namespace jr::mw {

InProcessMiddleware::InProcessMiddleware()
    : next_subscription_id_(1)
    , is_shutdown_(false) {
}

InProcessMiddleware::~InProcessMiddleware() {
  shutdown();
}

Subscription InProcessMiddleware::do_subscribe_typed(
  const std::string& topic,
  const std::string& type_full_name,
  std::function<void(const google::protobuf::Message&)> callback
) {
  std::unique_lock<std::mutex> lock(mutex_);

  const std::uint64_t id = next_subscription_id_++;
  TopicState& state = topics_[topic];

  // Enforce a single message type per topic
  if (!state.has_type) {
    state.type_full_name = type_full_name;
    state.has_type = true;
  } else if (state.type_full_name != type_full_name) {
    return Subscription{};
  }

  ensure_dispatcher(topic, state);
  state.subscriptions_typed.push_back(
    SubscriptionRecordTyped{id, type_full_name, std::move(callback)}
  );
  subscription_to_topic_[id] = topic;

  return make_subscription(id);
}

Subscription InProcessMiddleware::do_subscribe_any(
  const std::string& topic,
  std::function<void(const std::string&, const google::protobuf::Message&)>
    callback
) {
  std::unique_lock<std::mutex> lock(mutex_);

  const std::uint64_t id = next_subscription_id_++;

  TopicState& state = topics_[topic];
  ensure_dispatcher(topic, state);
  state.subscriptions_any.push_back(
    SubscriptionRecordAny{id, std::move(callback)}
  );
  subscription_to_topic_[id] = topic;

  return make_subscription(id);
}

void InProcessMiddleware::do_publish_serialized(
  const std::string& topic,
  const std::string& type_full_name,
  std::string payload
) {
  std::unique_lock<std::mutex> lock(mutex_);

  TopicState& state = topics_[topic];
  if (!state.has_type) {
    state.type_full_name = type_full_name;
    state.has_type = true;
  } else if (state.type_full_name != type_full_name) {
    return; // Drop mismatched publishes
  }

  while (state.queue.size() >= max_queue_size_) {
    state.queue.pop_front();
  }

  EncodedMessage em;
  em.type_full_name = type_full_name;
  em.payload = std::move(payload);
  em.timestamp_ns = now_ns();
  state.queue.emplace_back(std::move(em));
  state.cv.notify_one();
}

void InProcessMiddleware::do_unsubscribe(std::uint64_t id) {
  std::unique_lock<std::mutex> lock(mutex_);

  auto it = subscription_to_topic_.find(id);
  if (it == subscription_to_topic_.end()) {
    return;
  }

  const std::string topic = it->second;
  subscription_to_topic_.erase(it);
  TopicState& state = topics_[topic];

  // Remove from typed list
  for (auto sub_it = state.subscriptions_typed.begin();
       sub_it != state.subscriptions_typed.end();
       ++sub_it) {
    if (sub_it->id == id) {
      state.subscriptions_typed.erase(sub_it);
      break;
    }
  }

  // Remove from any list
  for (auto sub_it = state.subscriptions_any.begin();
       sub_it != state.subscriptions_any.end();
       ++sub_it) {
    if (sub_it->id == id) {
      state.subscriptions_any.erase(sub_it);
      break;
    }
  }

  if (state.subscriptions_typed.empty() && state.subscriptions_any.empty()) {
    state.is_active = false;
    state.cv.notify_all();
  }
}

void InProcessMiddleware::shutdown() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  is_shutdown_ = true;
  for (auto& [_, state] : topics_) {
    state.is_active = false;
    state.cv.notify_all();
  }
  lock.unlock();

  for (auto& [_, state] : topics_) {
    if (state.dispatcher.joinable())
      state.dispatcher.join();
  }
}

std::vector<TopicInfo> InProcessMiddleware::get_topic_names_and_types() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TopicInfo> out;
  out.reserve(topics_.size());
  for (const auto& kv : topics_) {
    const auto& name = kv.first;
    const auto& st = kv.second;
    out.push_back(
      TopicInfo{name, st.has_type ? st.type_full_name : std::string{}}
    );
  }
  return out;
}

std::uint64_t InProcessMiddleware::now_ns() {
  using namespace std::chrono;
  auto tp = system_clock::now().time_since_epoch();
  return duration_cast<nanoseconds>(tp).count();
}

std::unique_ptr<google::protobuf::Message> InProcessMiddleware::decode_message(
  const std::string& type_full_name,
  const std::string& payload
) {
  const auto* desc =
    google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(
      type_full_name
    );
  if (!desc)
    return {};

  const auto* prototype =
    google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc);
  if (!prototype)
    return {};

  std::unique_ptr<google::protobuf::Message> msg(prototype->New());
  if (!msg->ParseFromString(payload))
    return {};

  return msg;
}

void InProcessMiddleware::ensure_dispatcher(
  const std::string& topic,
  TopicState& state
) {
  if (!state.dispatcher.joinable()) {
    state.is_active = true;
    state.dispatcher = std::thread([this, topic]() { dispatch_loop(topic); });
  }
}

void InProcessMiddleware::dispatch_loop(std::string topic) {
  for (;;) {
    std::unique_lock<std::mutex> lock(mutex_);

    TopicState& state = topics_[topic];
    state.cv.wait(lock, [&]() {
      return !state.is_active || !state.queue.empty() || is_shutdown_;
    });

    if (!state.is_active && state.queue.empty()) {
      break;
    }

    if (state.queue.empty()) {
      continue;
    }

    auto em = state.queue.front();
    state.queue.pop_front();
    auto subs_typed = state.subscriptions_typed;
    auto subs_any = state.subscriptions_any;

    lock.unlock();

    auto msg = decode_message(em.type_full_name, em.payload);
    if (!msg) {
      continue;
    }

    for (auto& rec : subs_typed) {
      if (rec.type_full_name == em.type_full_name) {
        try {
          rec.callback(*msg);
        } catch (...) {
        }
      }
    }

    for (auto& rec : subs_any) {
      try {
        rec.callback(em.type_full_name, *msg);
      } catch (...) {
      }
    }
  }
}

} // namespace jr::mw
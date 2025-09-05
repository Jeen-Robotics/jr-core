#include "middleware/middleware.hpp"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace jr::mw {

namespace {

struct SubscriptionRecord {
  std::uint64_t id;
  std::type_index type;
  std::function<void(std::shared_ptr<void>)> callback;
};

class InProcessMiddleware final : public Middleware {
public:
  InProcessMiddleware()
      : next_subscription_id_(1)
      , is_shutdown_(false) {
  }
  ~InProcessMiddleware() override {
    shutdown();
  }

  Subscription do_subscribe(
    const std::string& topic,
    std::type_index type,
    std::function<void(std::shared_ptr<void>)> callback
  ) override {
    std::unique_lock<std::mutex> lock(mutex_);
    const std::uint64_t id = next_subscription_id_++;
    TopicState& state = topics_[topic];
    // Enforce a single message type per topic
    if (!state.has_type) {
      state.topic_type = type;
      state.has_type = true;
    } else if (state.topic_type != type) {
      // Reject mismatched subscription
      return Subscription{};
    }
    if (!state.dispatcher.joinable()) {
      state.is_active = true;
      state.dispatcher = std::thread([this, topic]() { dispatch_loop(topic); });
    }
    state.subscriptions.push_back(
      SubscriptionRecord{id, type, std::move(callback)}
    );
    subscription_to_topic_[id] = topic;
    return make_subscription(id);
  }

  void do_publish(
    const std::string& topic,
    std::type_index type,
    std::shared_ptr<void> payload
  ) override {
    std::unique_lock<std::mutex> lock(mutex_);
    TopicState& state = topics_[topic];
    // Enforce a single message type per topic
    if (!state.has_type) {
      state.topic_type = type;
      state.has_type = true;
    } else if (state.topic_type != type) {
      // Drop mismatched publishes
      return;
    }
    // Keep-latest policy: bound queue to size 1 by dropping oldest entries
    while (state.queue.size() >= max_queue_size_) {
      state.queue.pop_front();
    }
    state.queue.emplace_back(type, std::move(payload));
    state.cv.notify_one();
  }

  void do_unsubscribe(std::uint64_t id) override {
    std::unique_lock<std::mutex> lock(mutex_);
    auto it = subscription_to_topic_.find(id);
    if (it == subscription_to_topic_.end())
      return;
    const std::string topic = it->second;
    subscription_to_topic_.erase(it);
    TopicState& state = topics_[topic];
    for (auto sub_it = state.subscriptions.begin();
         sub_it != state.subscriptions.end();
         ++sub_it) {
      if (sub_it->id == id) {
        state.subscriptions.erase(sub_it);
        break;
      }
    }
    if (state.subscriptions.empty()) {
      state.is_active = false;
      state.cv.notify_all();
    }
  }

  void shutdown() override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (is_shutdown_)
      return;
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

private:
  struct TopicState {
    std::deque<std::pair<std::type_index, std::shared_ptr<void>>> queue;
    std::vector<SubscriptionRecord> subscriptions;
    std::condition_variable cv;
    std::mutex topic_mutex; // not used; using global mutex for simplicity
    std::thread dispatcher;
    bool is_active{false};
    // Single-type enforcement
    std::type_index topic_type{typeid(void)};
    bool has_type{false};
  };

  void dispatch_loop(std::string topic) {
    for (;;) {
      std::unique_lock<std::mutex> lock(mutex_);
      TopicState& state = topics_[topic];
      state.cv.wait(lock, [&]() {
        return !state.is_active || !state.queue.empty() || is_shutdown_;
      });
      if (!state.is_active && state.queue.empty()) {
        break;
      }
      if (state.queue.empty())
        continue;
      auto [msg_type, payload] = state.queue.front();
      state.queue.pop_front();
      auto subscribers = state.subscriptions; // copy under lock
      lock.unlock();
      for (auto& rec : subscribers) {
        if (rec.type == msg_type) {
          try {
            rec.callback(payload);
          } catch (...) { /* swallow to keep dispatcher alive */
          }
        }
      }
    }
  }

  std::mutex mutex_;
  std::unordered_map<std::string, TopicState> topics_;
  std::unordered_map<std::uint64_t, std::string> subscription_to_topic_;
  std::uint64_t next_subscription_id_;
  bool is_shutdown_;
  static constexpr std::size_t max_queue_size_ = 1; // keep-latest policy
};

} // namespace

namespace {
struct RuntimeState {
  std::mutex mutex;
  std::condition_variable cv;
  bool running{false};
  std::shared_ptr<Middleware> mw;
  BackendKind backend{BackendKind::InProcess};
  std::vector<std::shared_ptr<Node>> nodes;
};

RuntimeState& runtime() {
  static RuntimeState s;
  return s;
}
} // namespace

std::shared_ptr<Middleware> Middleware::create(BackendKind backend) {
  switch (backend) {
  case BackendKind::InProcess:
    return std::make_shared<InProcessMiddleware>();
  default:
    return std::make_shared<InProcessMiddleware>();
  }
}

std::shared_ptr<Middleware> get(BackendKind backend) {
  auto& rt = runtime();
  std::lock_guard<std::mutex> lock(rt.mutex);
  if (!rt.mw) {
    rt.mw = Middleware::create(backend);
    rt.backend = backend;
  }
  return rt.mw;
}

void init(BackendKind backend) {
  auto& rt = runtime();
  std::unique_lock<std::mutex> lock(rt.mutex);
  if (rt.mw) {
    // Fresh re-init: stop previous middleware first
    rt.mw->shutdown();
    rt.mw.reset();
  }
  rt.mw = Middleware::create(backend);
  rt.backend = backend;
  rt.running = true;
}

void spin(std::shared_ptr<Node> node) {
  auto& rt = runtime();
  std::unique_lock<std::mutex> lock(rt.mutex);
  rt.nodes.clear();
  if (node) rt.nodes.push_back(std::move(node));
  rt.cv.wait(lock, [&]() { return !rt.running; });
}

void spin(const std::vector<std::shared_ptr<Node>>& nodes) {
  auto& rt = runtime();
  std::unique_lock<std::mutex> lock(rt.mutex);
  rt.nodes = nodes; // keep them alive while spinning
  rt.cv.wait(lock, [&]() { return !rt.running; });
}

void shutdown() {
  auto& rt = runtime();
  std::unique_lock<std::mutex> lock(rt.mutex);
  if (rt.mw) {
    rt.mw->shutdown();
    rt.mw.reset();
  }
  rt.running = false;
  rt.nodes.clear();
  lock.unlock();
  rt.cv.notify_all();
}

} // namespace jr::mw
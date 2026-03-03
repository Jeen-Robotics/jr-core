#include <middleware/backend.hpp>
#include <middleware/detail/transport.hpp>

#include <middleware_rs/middleware.hpp>
#include <middleware_rs/publisher.hpp>
#include <middleware_rs/subscriber.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace jr::mw::detail {

namespace {

Backend default_backend() {
#if defined(JR_MIDDLEWARE_DEFAULT_BACKEND_CPP) &&                              \
  JR_MIDDLEWARE_DEFAULT_BACKEND_CPP
  return Backend::Cpp;
#else
  return Backend::Rust;
#endif
}

std::optional<Backend> parse_backend_string(const char* value) {
  if (!value) {
    return std::nullopt;
  }

  std::string normalized(value);
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); }
  );

  if (normalized == "rust") {
    return Backend::Rust;
  }
  if (normalized == "cpp" || normalized == "c++") {
    return Backend::Cpp;
  }

  return std::nullopt;
}

std::mutex g_backend_mutex;
Backend g_backend_config = default_backend();
bool g_backend_explicit = false;
bool g_backend_locked = false;

Backend effective_backend_locked() {
  if (g_backend_explicit) {
    return g_backend_config;
  }

  const auto env_backend =
    parse_backend_string(std::getenv("JR_MIDDLEWARE_BACKEND"));
  if (env_backend.has_value()) {
    return *env_backend;
  }

  return g_backend_config;
}

Backend lock_backend_for_use() {
  std::lock_guard<std::mutex> lock(g_backend_mutex);
  const Backend backend = effective_backend_locked();
  g_backend_config = backend;
  g_backend_locked = true;
  return backend;
}

namespace cpp_backend {

struct QueueState {
  std::mutex mutex;
  std::deque<std::vector<std::uint8_t>> queue;
  bool closed = false;
  Qos qos = Qos::KeepLast;
  std::size_t capacity = 16;
};

struct TopicState {
  std::vector<std::weak_ptr<QueueState>> subscribers;
};

class Bus {
public:
  static Bus& instance() {
    static Bus bus;
    return bus;
  }

  std::shared_ptr<QueueState> add_subscriber(
    const std::string& topic,
    Qos qos,
    std::size_t capacity
  ) {
    auto state = std::make_shared<QueueState>();
    state->qos = qos;
    state->capacity = std::max<std::size_t>(1, capacity);
    if (qos == Qos::SensorData) {
      state->capacity = 1;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    topics_[topic].subscribers.emplace_back(state);
    return state;
  }

  void remove_subscriber(
    const std::string& topic,
    const std::shared_ptr<QueueState>& state
  ) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto topic_it = topics_.find(topic);
    if (topic_it == topics_.end()) {
      return;
    }

    auto& subs = topic_it->second.subscribers;
    subs.erase(
      std::remove_if(
        subs.begin(),
        subs.end(),
        [&](const std::weak_ptr<QueueState>& weak_state) {
          const auto locked = weak_state.lock();
          return !locked || locked == state;
        }
      ),
      subs.end()
    );

    if (subs.empty()) {
      topics_.erase(topic_it);
    }
  }

  bool publish(
    const std::string& topic,
    const std::uint8_t* data,
    std::size_t len
  ) {
    std::vector<std::shared_ptr<QueueState>> subscribers;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto topic_it = topics_.find(topic);
      if (topic_it == topics_.end()) {
        return true;
      }

      auto& subs = topic_it->second.subscribers;
      subscribers.reserve(subs.size());

      subs.erase(
        std::remove_if(
          subs.begin(),
          subs.end(),
          [&](const std::weak_ptr<QueueState>& weak_state) {
            const auto state = weak_state.lock();
            if (!state) {
              return true;
            }
            subscribers.push_back(state);
            return false;
          }
        ),
        subs.end()
      );
    }

    for (const auto& state : subscribers) {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->closed) {
        continue;
      }

      if (state->qos == Qos::SensorData) {
        state->queue.clear();
      } else if (state->queue.size() >= state->capacity) {
        state->queue.pop_front();
      }

      state->queue.emplace_back(data, data + len);
    }

    return true;
  }

  bool topic_exists(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic);
    return it != topics_.end() && !it->second.subscribers.empty();
  }

  std::size_t subscriber_count(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      return 0;
    }

    std::size_t count = 0;
    for (const auto& weak_state : it->second.subscribers) {
      if (weak_state.lock()) {
        ++count;
      }
    }
    return count;
  }

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicState> topics_;
};

} // namespace cpp_backend

class RustPublisherHandle final : public PublisherHandle {
public:
  explicit RustPublisherHandle(PublisherPtr impl)
      : impl_(std::move(impl)) {
  }

  bool publish(const void* data, std::size_t len) override {
    return impl_ && publish_impl(impl_.get(), data, len);
  }

  bool valid() const override {
    return impl_ && publisher_valid_impl(impl_.get());
  }

private:
  PublisherPtr impl_;
};

class RustSubscriberHandle final : public SubscriberHandle {
public:
  explicit RustSubscriberHandle(SubscriberPtr impl)
      : impl_(std::move(impl)) {
  }

  bool valid() const override {
    return impl_ && subscriber_valid_impl(impl_.get());
  }

  int get_fd() const override {
    return impl_ ? subscriber_fd_impl(impl_.get()) : -1;
  }

  TransportRecvResult recv() override {
    if (!impl_) {
      return {false, {}, true};
    }

    const auto result = subscriber_recv_impl(impl_.get());
    return {result.has_message, result.data, result.closed};
  }

private:
  SubscriberPtr impl_;
};

class CppPublisherHandle final : public PublisherHandle {
public:
  explicit CppPublisherHandle(std::string topic)
      : topic_(std::move(topic)) {
  }

  bool publish(const void* data, std::size_t len) override {
    return cpp_backend::Bus::instance()
      .publish(topic_, static_cast<const std::uint8_t*>(data), len);
  }

  bool valid() const override {
    return true;
  }

private:
  std::string topic_;
};

class CppSubscriberHandle final : public SubscriberHandle {
public:
  CppSubscriberHandle(std::string topic, Qos qos, std::size_t capacity)
      : topic_(std::move(topic))
      , state_(
          cpp_backend::Bus::instance().add_subscriber(topic_, qos, capacity)
        ) {
  }

  ~CppSubscriberHandle() override {
    if (state_) {
      {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->closed = true;
        state_->queue.clear();
      }
      cpp_backend::Bus::instance().remove_subscriber(topic_, state_);
    }
  }

  bool valid() const override {
    if (!state_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(state_->mutex);
    return !state_->closed;
  }

  int get_fd() const override {
    return -1;
  }

  TransportRecvResult recv() override {
    if (!state_) {
      return {false, {}, true};
    }

    std::lock_guard<std::mutex> lock(state_->mutex);
    TransportRecvResult result{false, {}, state_->closed};
    if (state_->queue.empty()) {
      return result;
    }
    result.has_message = true;
    result.data = std::move(state_->queue.front());
    state_->queue.pop_front();
    return result;
  }

private:
  std::string topic_;
  std::shared_ptr<cpp_backend::QueueState> state_;
};

} // namespace

bool init_backend() {
  const Backend backend = lock_backend_for_use();
  if (backend == Backend::Rust) {
    return jr::mw::rs::init();
  }
  return true;
}

bool set_backend_impl(Backend backend) {
  std::lock_guard<std::mutex> lock(g_backend_mutex);
  const Backend current = effective_backend_locked();
  if (g_backend_locked && current != backend) {
    return false;
  }
  g_backend_explicit = true;
  g_backend_config = backend;
  return true;
}

Backend get_backend_impl() {
  std::lock_guard<std::mutex> lock(g_backend_mutex);
  return effective_backend_locked();
}

const char* backend_name_impl(Backend backend) {
  switch (backend) {
  case Backend::Rust:
    return "rust";
  case Backend::Cpp:
    return "cpp";
  default:
    return "unknown";
  }
}

std::shared_ptr<PublisherHandle> create_publisher_handle(
  const std::string& topic,
  Qos qos,
  std::size_t capacity
) {
  if (!init_backend()) {
    return {};
  }

  const Backend backend = lock_backend_for_use();
  if (backend == Backend::Cpp) {
    return std::make_shared<CppPublisherHandle>(topic);
  }

  auto impl = create_publisher_impl(topic, qos, capacity);
  if (!impl) {
    return {};
  }
  return std::make_shared<RustPublisherHandle>(std::move(impl));
}

bool publish_handle(
  const std::shared_ptr<PublisherHandle>& handle,
  const void* data,
  std::size_t len
) {
  return handle && handle->publish(data, len);
}

bool publisher_valid(const std::shared_ptr<PublisherHandle>& handle) {
  return handle && handle->valid();
}

std::shared_ptr<SubscriberHandle> create_subscriber_handle(
  const std::string& topic,
  Qos qos,
  std::size_t capacity
) {
  if (!init_backend()) {
    return {};
  }

  const Backend backend = lock_backend_for_use();
  if (backend == Backend::Cpp) {
    return std::make_shared<CppSubscriberHandle>(topic, qos, capacity);
  }

  auto impl = create_subscriber_impl(topic, qos, capacity);
  if (!impl) {
    return {};
  }
  return std::make_shared<RustSubscriberHandle>(std::move(impl));
}

bool subscriber_valid(const std::shared_ptr<SubscriberHandle>& handle) {
  return handle && handle->valid();
}

int subscriber_fd(const std::shared_ptr<SubscriberHandle>& handle) {
  return handle ? handle->get_fd() : -1;
}

TransportRecvResult subscriber_recv(
  const std::shared_ptr<SubscriberHandle>& handle
) {
  if (!handle) {
    return {false, {}, true};
  }
  return handle->recv();
}

bool topic_exists_impl(const std::string& topic) {
  const Backend backend = lock_backend_for_use();
  if (backend == Backend::Cpp) {
    return cpp_backend::Bus::instance().topic_exists(topic);
  }
  return jr::mw::rs::topic_exists(topic);
}

std::size_t subscriber_count_impl(const std::string& topic) {
  const Backend backend = lock_backend_for_use();
  if (backend == Backend::Cpp) {
    return cpp_backend::Bus::instance().subscriber_count(topic);
  }
  return jr::mw::rs::subscriber_count(topic);
}

} // namespace jr::mw::detail

namespace jr::mw {

bool set_backend(Backend backend) {
  return detail::set_backend_impl(backend);
}

Backend get_backend() {
  return detail::get_backend_impl();
}

const char* backend_name(Backend backend) {
  return detail::backend_name_impl(backend);
}

bool init() {
  return detail::init_backend();
}

} // namespace jr::mw

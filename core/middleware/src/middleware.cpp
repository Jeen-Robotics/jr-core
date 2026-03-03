#include <middleware/detail/subscription_any_impl.hpp>
#include <middleware/detail/subscription_base.hpp>
#include <middleware/detail/subscription_impl.hpp>
#include <middleware/detail/transport.hpp>
#include <middleware/middleware.hpp>
#include <middleware/node.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#ifdef __linux__
#include <sys/epoll.h>
#include <unistd.h>
#endif

#include <chrono>
#include <iostream>

namespace jr::mw {

namespace {

// Global middleware instance
std::shared_ptr<Middleware> g_middleware;
std::mutex g_mutex;
std::atomic<bool> g_shutdown_requested{false};

// Condition variable for spin() to avoid busy-waiting
std::mutex g_spin_mutex;
std::condition_variable g_spin_cv;

} // namespace

// --- Subscription ---

Subscription::Subscription(std::weak_ptr<Middleware> owner, std::uint64_t id)
    : owner_(std::move(owner))
    , id_(id) {
}

Subscription::Subscription(Subscription&& other) noexcept
    : owner_(std::move(other.owner_))
    , id_(other.id_) {
  other.id_ = 0;
}

Subscription& Subscription::operator=(Subscription&& other) noexcept {
  if (this != &other) {
    unsubscribe();
    owner_ = std::move(other.owner_);
    id_ = other.id_;
    other.id_ = 0;
  }
  return *this;
}

Subscription::~Subscription() {
  unsubscribe();
}

void Subscription::unsubscribe() {
  if (id_ != 0) {
    if (auto owner = owner_.lock()) {
      owner->unregister_subscription(id_);
    }
    id_ = 0;
  }
}

bool Subscription::valid() const noexcept {
  if (id_ == 0)
    return false;
  auto owner = owner_.lock();
  if (!owner)
    return false;
  return owner->subscription_valid(id_);
}

// --- Middleware ---

Middleware::Middleware()
    : init_failed_(false)
#ifdef __linux__
    , epoll_fd_(-1)
#endif
{
  // Initialize selected backend (idempotent)
  if (!jr::mw::init()) {
    std::cerr << "[jr::mw] ERROR: Failed to initialize middleware backend"
              << std::endl;
    init_failed_ = true;
    return;
  }

#ifdef __linux__
  int fd = ::epoll_create1(0);
  if (fd < 0) {
    std::cerr << "[jr::mw] WARNING: Failed to create epoll instance, falling "
                 "back to poll"
              << std::endl;
  }
  epoll_fd_.store(fd);
#endif
}

Middleware::~Middleware() {
  shutdown();
}

void Middleware::shutdown() {
  // Guard against double shutdown
  bool expected = false;
  if (!shutdown_.compare_exchange_strong(expected, true)) {
    return; // Already shutdown
  }

  // Acquire mutex to synchronize with ensure_dispatcher()
  // This prevents the race where:
  // 1. ensure_dispatcher() checks shutdown_ (false)
  // 2. shutdown() sets shutdown_ = true, checks joinable() = false
  // 3. ensure_dispatcher() starts thread
  // 4. Thread runs against destroyed Middleware
  std::thread dispatcher_to_join;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dispatcher_to_join = std::move(dispatcher_);
    subscriptions_.clear();
    publishers_.clear();
#ifdef __linux__
    fd_to_id_.clear();
    int fd = epoll_fd_.load();
    if (fd >= 0) {
      ::close(fd);
      epoll_fd_.store(-1);
    }
#endif
  }

  if (dispatcher_to_join.joinable()) {
    dispatcher_to_join.join();
  }
}

void Middleware::publish(
  const std::string& topic,
  const google::protobuf::Message& message,
  Qos qos,
  std::size_t capacity
) {
  // Get type name - descriptor should never be null for properly generated
  // protos
  const auto* desc = message.GetDescriptor();
  if (!desc) {
    std::cerr << "[jr::mw] WARNING: Message has no descriptor for topic: "
              << topic << std::endl;
    return;
  }

  auto sv = desc->full_name();
  std::string type_full_name(sv.data(), sv.size());

  // Serialize and publish
  std::string data;
  if (!message.SerializeToString(&data)) {
    std::cerr << "[jr::mw] WARNING: Failed to serialize message for topic: "
              << topic << std::endl;
    return;
  }

  publish_serialized(topic, type_full_name, data, qos, capacity);
}

void Middleware::publish_serialized(
  const std::string& topic,
  const std::string& type_full_name,
  const std::string& payload,
  Qos qos,
  std::size_t capacity
) {
  // Check shutdown first
  if (shutdown_.load()) {
    return;
  }

  // Get or create cached publisher under lock, keep shared_ptr alive
  std::shared_ptr<detail::PublisherHandle> pub_shared;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Re-check shutdown under lock to close race window
    if (shutdown_.load()) {
      return;
    }

    // Type check
    auto type_it = topic_types_.find(topic);
    if (type_it == topic_types_.end()) {
      topic_types_[topic] = type_full_name;
    } else if (type_it->second != type_full_name) {
      std::cerr << "[jr::mw] WARNING: Type mismatch for topic " << topic
                << ": expected " << type_it->second << ", got "
                << type_full_name << std::endl;
      return;
    }

    Qos effective_qos = qos;
    std::size_t effective_capacity = capacity;
    auto pref_it = topic_qos_preferences_.find(topic);
    if (pref_it != topic_qos_preferences_.end() &&
        pref_it->second == Qos::SensorData) {
      effective_qos = Qos::SensorData;
      effective_capacity = 1;
    }

    // Get or create cached publisher with specified QoS
    auto pub_it = publishers_.find(topic);
    const bool needs_upgrade =
      (pub_it != publishers_.end() && pub_it->second.qos != Qos::SensorData &&
       effective_qos == Qos::SensorData);

    if (pub_it == publishers_.end() || needs_upgrade) {
      auto new_pub = detail::create_publisher_handle(
        topic,
        effective_qos,
        effective_capacity
      );
      if (!new_pub) {
        return;
      }
      pub_shared = std::move(new_pub);
      publishers_[topic] = {pub_shared, effective_qos, effective_capacity};
    } else {
      // Warn if QoS parameters differ from cached publisher
      auto& entry = pub_it->second;
      if (entry.qos != effective_qos || entry.capacity != effective_capacity) {
        std::cerr
          << "[jr::mw] WARNING: QoS mismatch for cached publisher on topic "
          << topic << ": using cached (qos=" << static_cast<int>(entry.qos)
          << ", capacity=" << entry.capacity << ")"
          << ", ignoring requested (qos=" << static_cast<int>(effective_qos)
          << ", capacity=" << effective_capacity << ")" << std::endl;
      }
      pub_shared = entry.impl;
    }
  }

  // Publish outside the lock - pub_shared keeps publisher alive
  if (pub_shared) {
    detail::publish_handle(pub_shared, payload.data(), payload.size());

    // Notify dispatcher on non-Linux platforms
#ifndef __linux__
    dispatch_cv_.notify_one();
#endif
  }
}

Subscription Middleware::subscribe_any(
  const std::string& topic,
  std::function<void(const std::string&, const google::protobuf::Message&)>
    callback,
  Qos qos,
  std::size_t capacity
) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto qos_it = topic_qos_preferences_.find(topic);
    if (qos_it == topic_qos_preferences_.end() || qos == Qos::SensorData) {
      topic_qos_preferences_[topic] = qos;
    }
  }

  // Create raw bytes subscriber from selected backend
  auto sub_impl = detail::create_subscriber_handle(topic, qos, capacity);
  if (!sub_impl || !detail::subscriber_valid(sub_impl)) {
    std::cerr << "[jr::mw] WARNING: Failed to create subscriber for topic: "
              << topic << std::endl;
    return Subscription{};
  }

  // Allocate ID
  std::uint64_t id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    id = next_id_++;
  }

  // Create type-erased subscription that uses protobuf reflection
  // Pass references to mutex_ and topic_types_ for dynamic type lookup
  auto impl = std::make_shared<detail::SubscriptionAnyImpl>(
    std::move(sub_impl),
    topic,
    std::move(callback),
    mutex_,
    topic_types_
  );

  return make_subscription(id, std::move(impl));
}

std::vector<TopicInfo> Middleware::get_topic_names_and_types() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TopicInfo> result;
  result.reserve(topic_types_.size());
  for (const auto& [name, type] : topic_types_) {
    result.push_back({name, type});
  }
  return result;
}

void Middleware::register_topic_type(
  const std::string& topic,
  const std::string& type_full_name
) {
  if (type_full_name.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = topic_types_.find(topic);
  if (it == topic_types_.end()) {
    topic_types_[topic] = type_full_name;
  }
  // If type already registered, ignore (first publisher wins)
}

std::shared_ptr<Middleware> Middleware::create() {
  auto mw = std::shared_ptr<Middleware>(new Middleware());
  if (mw->init_failed_) {
    return nullptr;
  }
  return mw;
}

Subscription Middleware::make_subscription(
  std::uint64_t id,
  std::shared_ptr<detail::SubscriptionBase> impl
) {
  {
    std::lock_guard<std::mutex> lock(mutex_);

#ifdef __linux__
    // Add eventfd to epoll for O(1) event dispatch
    int epoll_fd = epoll_fd_.load();
    if (epoll_fd >= 0) {
      int fd = impl->get_fd();
      if (fd >= 0) {
        struct epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.u64 = id;
        if (::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) == 0) {
          fd_to_id_[fd] = id;
        }
      }
    }
#endif
    subscriptions_[id] = std::move(impl);
  }

  ensure_dispatcher();
  return Subscription{weak_from_this(), id};
}

void Middleware::unregister_subscription(std::uint64_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = subscriptions_.find(id);
  if (it != subscriptions_.end()) {
#ifdef __linux__
    // Remove eventfd from epoll
    int epoll_fd = epoll_fd_.load();
    if (epoll_fd >= 0 && it->second) {
      int fd = it->second->get_fd();
      if (fd >= 0) {
        ::epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, nullptr);
        fd_to_id_.erase(fd);
      }
    }
#endif
    // IMPORTANT: Order matters here for thread safety:
    // 1. cancel() sets atomic cancelled_ flag
    // 2. erase() removes from map
    //
    // The dispatcher thread may already have a shared_ptr copy from
    // before this lock was acquired. The cancel() ensures spin_once()
    // will return false even if the dispatcher tries to invoke it.
    // The shared_ptr keeps the object alive until dispatcher releases it.
    it->second->cancel();
    subscriptions_.erase(it);
  }
}

bool Middleware::subscription_valid(std::uint64_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = subscriptions_.find(id);
  if (it == subscriptions_.end())
    return false;
  return it->second && it->second->valid();
}

void Middleware::dispatcher_loop() {
  while (!shutdown_.load()) {
#ifdef __linux__
    int epoll_fd = epoll_fd_.load();
    if (epoll_fd < 0) {
      // Fallback: no epoll available, poll all subscriptions
      std::vector<std::shared_ptr<detail::SubscriptionBase>> subs_to_process;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [id, sub] : subscriptions_) {
          if (sub) {
            subs_to_process.push_back(sub);
          }
        }
      }

      bool processed_any = false;
      for (auto& sub : subs_to_process) {
        while (sub->spin_once()) {
          processed_any = true;
        }
      }

      if (processed_any) {
        g_spin_cv.notify_all();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // Wait for events with epoll (O(1) per event, not O(N) per iteration)
    constexpr int MAX_EVENTS = 64;
    struct epoll_event events[MAX_EVENTS];
    int nfds = ::epoll_wait(epoll_fd, events, MAX_EVENTS, 10); // 10ms timeout

    // Collect subscriptions to process (acquire lock once)
    std::vector<std::shared_ptr<detail::SubscriptionBase>> subs_to_process;
    if (nfds > 0) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (int i = 0; i < nfds; ++i) {
        std::uint64_t id = events[i].data.u64;
        auto it = subscriptions_.find(id);
        if (it != subscriptions_.end() && it->second) {
          subs_to_process.push_back(it->second); // shared_ptr copy
        }
      }
    }

    // Process subscriptions outside the lock
    bool processed_any = false;
    for (auto& sub : subs_to_process) {
      while (sub->spin_once()) {
        processed_any = true;
      }
    }

    // Backends without fd support (get_fd() < 0) cannot wake epoll.
    // Poll them explicitly every loop.
    std::vector<std::shared_ptr<detail::SubscriptionBase>> poll_only_subs;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& [id, sub] : subscriptions_) {
        if (sub && sub->get_fd() < 0) {
          poll_only_subs.push_back(sub);
        }
      }
    }

    for (auto& sub : poll_only_subs) {
      while (sub->spin_once()) {
        processed_any = true;
      }
    }

    // Notify spin() that callbacks were processed
    if (processed_any) {
      g_spin_cv.notify_all();
    }
#else
    // Non-Linux: use condition variable with timeout
    // Note: We poll every 10ms because the predicate only checks shutdown.
    // The notify_one() from publish() reduces latency when messages arrive.
    std::vector<std::shared_ptr<detail::SubscriptionBase>> subs_to_process;
    {
      std::unique_lock<std::mutex> lock(mutex_);

      // Wait for shutdown signal or timeout (10ms max for message polling)
      dispatch_cv_.wait_for(lock, std::chrono::milliseconds(10), [this]() {
        return shutdown_.load();
      });

      if (shutdown_.load()) {
        break;
      }

      for (auto& [id, sub] : subscriptions_) {
        if (sub) {
          subs_to_process.push_back(sub); // shared_ptr copy
        }
      }
    }

    // Process outside the lock - shared_ptrs keep objects alive
    bool processed_any = false;
    for (auto& sub : subs_to_process) {
      while (sub->spin_once()) {
        processed_any = true;
      }
    }

    // Notify spin() that callbacks were processed
    if (processed_any) {
      g_spin_cv.notify_all();
    }
#endif
  }

  // Reset flag so dispatcher can be restarted if needed
  dispatcher_running_.store(false);
}

void Middleware::ensure_dispatcher() {
  // Quick check without lock - if already running or shutdown, nothing to do
  if (shutdown_.load() || dispatcher_running_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Re-check under lock to prevent race with shutdown()
  if (shutdown_.load() || dispatcher_running_.load()) {
    return;
  }

  dispatcher_running_.store(true);
  dispatcher_ = std::thread([this]() { dispatcher_loop(); });
}

// --- Global functions ---

std::shared_ptr<Middleware> get() {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_middleware) {
    g_middleware = Middleware::create();
    g_shutdown_requested.store(false);
  }
  return g_middleware;
}

void shutdown() {
  g_shutdown_requested.store(true);

  // Wake up any spin() loops waiting on condition variable
  g_spin_cv.notify_all();

  std::shared_ptr<Middleware> mw;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    mw = std::move(g_middleware);
  }

  if (mw) {
    mw->shutdown();
  }
}

void spin(const std::shared_ptr<Node>& node) {
  spin(std::vector<std::shared_ptr<Node>>{node});
}

/// @note spin() only exits when the global shutdown() is called (sets
/// g_shutdown_requested).
///       Calling mw->shutdown() on a specific Middleware instance does NOT
///       cause spin() to exit. This is intentional - spin() is designed for the
///       global middleware lifecycle.
void spin(const std::vector<std::shared_ptr<Node>>& nodes) {
  while (!g_shutdown_requested.load()) {
    for (const auto& node : nodes) {
      if (node) {
        node->spin_once();
      }
    }

    // Wait for dispatcher activity or timeout (avoids busy-waiting)
    std::unique_lock<std::mutex> lock(g_spin_mutex);
    g_spin_cv.wait_for(lock, std::chrono::milliseconds(10), []() {
      return g_shutdown_requested.load();
    });
  }
}

// --- Node ---

Node::Node(std::string node_name)
    : node_name_(std::move(node_name))
    , mw_(get()) {
}

Node::Node(std::string node_name, std::shared_ptr<Middleware> mw)
    : node_name_(std::move(node_name))
    , mw_(std::move(mw)) {
}

void Node::spin_once() {
  // Default implementation does nothing
}

const std::string& Node::name() const noexcept {
  return node_name_;
}

bool Node::valid() const noexcept {
  return mw_ != nullptr;
}

} // namespace jr::mw

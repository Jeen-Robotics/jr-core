#include <middleware/middleware.hpp>
#include <middleware/node.hpp>
#include <middleware/detail/subscription_base.hpp>
#include <middleware/detail/subscription_impl.hpp>

#include <middleware_rs/middleware.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#ifdef __linux__
#include <poll.h>
#endif

#include <chrono>
#include <condition_variable>

namespace jr::mw {

namespace {

// Global middleware instance
std::shared_ptr<Middleware> g_middleware;
std::mutex g_mutex;
std::atomic<bool> g_shutdown_requested{false};
std::condition_variable g_shutdown_cv;

} // namespace

// --- Subscription ---

Subscription::Subscription(std::weak_ptr<Middleware> owner, std::uint64_t id)
    : owner_(std::move(owner))
    , id_(id)
{}

Subscription::Subscription(Subscription&& other) noexcept
    : owner_(std::move(other.owner_))
    , id_(other.id_)
{
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
    if (id_ == 0) return false;
    auto owner = owner_.lock();
    if (!owner) return false;
    return owner->subscription_valid(id_);
}

// --- Middleware ---

Middleware::Middleware() {
    // Initialize Rust backend
    jr::mw::init();
}

Middleware::~Middleware() {
    shutdown();
}

void Middleware::shutdown() {
    shutdown_.store(true);
    
    if (dispatcher_.joinable()) {
        dispatcher_.join();
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    subscriptions_.clear();
}

void Middleware::publish(const std::string& topic, const google::protobuf::Message& message) {
    // Get type name
    std::string type_full_name;
    const auto* desc = message.GetDescriptor();
    if (desc) {
        auto sv = desc->full_name();
        type_full_name.assign(sv.data(), sv.size());
    }

    // Serialize and publish
    std::string data;
    if (!message.SerializeToString(&data)) {
        return;
    }

    publish_serialized(topic, type_full_name, data);
}

void Middleware::publish_serialized(
    const std::string& topic,
    const std::string& type_full_name,
    const std::string& payload
) {
    // Check/register type
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = topic_types_.find(topic);
        if (it == topic_types_.end()) {
            topic_types_[topic] = type_full_name;
        } else if (it->second != type_full_name) {
            // Type mismatch - silently ignore
            return;
        }
    }

    // Publish via Rust backend
    auto pub_impl = detail::create_publisher_impl(topic, Qos::KeepLast, 16);
    if (pub_impl) {
        detail::publish_impl(pub_impl.get(), payload.data(), payload.size());
    }
}

Subscription Middleware::subscribe(
    const std::string& topic,
    const std::string& type_full_name,
    std::function<void(const google::protobuf::Message&)> callback
) {
    // Dynamic subscription requires message factory
    // Not implemented yet - requires protobuf reflection
    (void)topic;
    (void)type_full_name;
    (void)callback;
    return Subscription{};
}

Subscription Middleware::subscribe_any(
    const std::string& topic,
    std::function<void(const std::string&, const google::protobuf::Message&)> callback
) {
    // Any subscription requires message factory
    // Not implemented yet - requires protobuf reflection
    (void)topic;
    (void)callback;
    return Subscription{};
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

std::shared_ptr<Middleware> Middleware::create() {
    return std::shared_ptr<Middleware>(new Middleware());
}

Subscription Middleware::make_subscription(
    std::uint64_t id,
    std::unique_ptr<detail::SubscriptionBase> impl
) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        subscriptions_[id] = std::move(impl);
    }
    
    ensure_dispatcher();
    return Subscription{weak_from_this(), id};
}

void Middleware::unregister_subscription(std::uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    subscriptions_.erase(id);
}

bool Middleware::subscription_valid(std::uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subscriptions_.find(id);
    if (it == subscriptions_.end()) return false;
    return it->second && it->second->valid();
}

void Middleware::dispatcher_loop() {
    while (!shutdown_.load()) {
#ifdef __linux__
        // Collect file descriptors for poll
        std::vector<std::pair<int, std::uint64_t>> fds;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& [id, sub] : subscriptions_) {
                if (sub) {
                    int fd = sub->get_fd();
                    if (fd >= 0) {
                        fds.emplace_back(fd, id);
                    }
                }
            }
        }

        if (fds.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Poll for events
        std::vector<struct pollfd> pfds(fds.size());
        for (std::size_t i = 0; i < fds.size(); ++i) {
            pfds[i].fd = fds[i].first;
            pfds[i].events = POLLIN;
            pfds[i].revents = 0;
        }

        int ret = ::poll(pfds.data(), static_cast<nfds_t>(pfds.size()), 10);  // 10ms timeout
        if (ret <= 0) {
            continue;
        }

        // Process ready subscriptions
        for (std::size_t i = 0; i < pfds.size(); ++i) {
            if (pfds[i].revents & POLLIN) {
                std::uint64_t id = fds[i].second;
                std::lock_guard<std::mutex> lock(mutex_);
                auto it = subscriptions_.find(id);
                if (it != subscriptions_.end() && it->second) {
                    // Drain all messages
                    while (it->second->spin_once()) {
                        // Continue until no more messages
                    }
                }
            }
        }
#else
        // Non-Linux: simple polling
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& [id, sub] : subscriptions_) {
                if (sub) {
                    while (sub->spin_once()) {
                        // Drain messages
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    }
}

void Middleware::ensure_dispatcher() {
    if (dispatcher_running_.load()) {
        return;
    }
    
    bool expected = false;
    if (dispatcher_running_.compare_exchange_strong(expected, true)) {
        dispatcher_ = std::thread([this]() {
            dispatcher_loop();
        });
    }
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
    g_shutdown_cv.notify_all();
    
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

void spin(const std::vector<std::shared_ptr<Node>>& nodes) {
    while (!g_shutdown_requested.load()) {
        for (const auto& node : nodes) {
            if (node) {
                node->spin_once();
            }
        }
        
        // Brief sleep to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// --- Node ---

Node::Node(std::string node_name)
    : node_name_(std::move(node_name))
    , mw_(get())
{}

Node::Node(std::string node_name, std::shared_ptr<Middleware> mw)
    : node_name_(std::move(node_name))
    , mw_(std::move(mw))
{}

void Node::spin_once() {
    // Default implementation does nothing
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
    std::function<void(const std::string&, const google::protobuf::Message&)> callback
) const {
    return mw_->subscribe_any(topic, std::move(callback));
}

bool Node::valid() const noexcept {
    return mw_ != nullptr;
}

} // namespace jr::mw

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
#include <iostream>

namespace jr::mw {

namespace {

// Global middleware instance
std::shared_ptr<Middleware> g_middleware;
std::mutex g_mutex;
std::atomic<bool> g_shutdown_requested{false};

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
    // Initialize Rust backend (idempotent - Rust uses OnceCell singleton)
    if (!jr::mw::init()) {
        std::cerr << "[jr::mw] FATAL: Failed to initialize Rust middleware backend" << std::endl;
        std::abort();
    }
}

Middleware::~Middleware() {
    shutdown();
}

void Middleware::shutdown() {
    // Guard against double shutdown
    bool expected = false;
    if (!shutdown_.compare_exchange_strong(expected, true)) {
        return;  // Already shutdown
    }
    
    if (dispatcher_.joinable()) {
        dispatcher_.join();
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    subscriptions_.clear();
    publishers_.clear();
}

void Middleware::publish(const std::string& topic, const google::protobuf::Message& message) {
    // Get type name - descriptor should never be null for properly generated protos
    const auto* desc = message.GetDescriptor();
    if (!desc) {
        std::cerr << "[jr::mw] WARNING: Message has no descriptor for topic: " << topic << std::endl;
        return;
    }
    
    auto sv = desc->full_name();
    std::string type_full_name(sv.data(), sv.size());

    // Serialize and publish
    std::string data;
    if (!message.SerializeToString(&data)) {
        std::cerr << "[jr::mw] WARNING: Failed to serialize message for topic: " << topic << std::endl;
        return;
    }

    publish_serialized(topic, type_full_name, data);
}

void Middleware::publish_serialized(
    const std::string& topic,
    const std::string& type_full_name,
    const std::string& payload
) {
    // Check shutdown first
    if (shutdown_.load()) {
        return;
    }

    // Get or create cached publisher under lock, keep shared_ptr alive
    std::shared_ptr<detail::PublisherImpl> pub_shared;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Type check
        auto type_it = topic_types_.find(topic);
        if (type_it == topic_types_.end()) {
            topic_types_[topic] = type_full_name;
        } else if (type_it->second != type_full_name) {
            std::cerr << "[jr::mw] WARNING: Type mismatch for topic " << topic 
                      << ": expected " << type_it->second << ", got " << type_full_name << std::endl;
            return;
        }
        
        // Get or create cached publisher
        auto pub_it = publishers_.find(topic);
        if (pub_it == publishers_.end()) {
            // Use SensorData QoS for better compatibility with realtime topics
            auto new_pub = detail::create_publisher_impl(topic, Qos::SensorData, 1);
            if (!new_pub) {
                return;
            }
            // Convert unique_ptr to shared_ptr for safe concurrent access
            pub_shared = std::shared_ptr<detail::PublisherImpl>(
                new_pub.release(), 
                [](detail::PublisherImpl* p) { detail::delete_publisher_impl(p); }
            );
            publishers_[topic] = pub_shared;
        } else {
            pub_shared = pub_it->second;
        }
    }

    // Publish outside the lock - pub_shared keeps publisher alive
    if (pub_shared) {
        detail::publish_impl(pub_shared.get(), payload.data(), payload.size());
    }
}

Subscription Middleware::subscribe_any(
    const std::string& topic,
    std::function<void(const std::string&, const google::protobuf::Message&)> callback
) {
    // Not implemented - requires protobuf reflection/dynamic message factory
    // Log warning once per topic
    std::cerr << "[jr::mw] WARNING: subscribe_any() not implemented for Rust backend. "
              << "Topic '" << topic << "' will not receive messages. "
              << "BagWriter recording will not work." << std::endl;
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
    std::shared_ptr<detail::SubscriptionBase> impl
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
    auto it = subscriptions_.find(id);
    if (it != subscriptions_.end()) {
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
    if (it == subscriptions_.end()) return false;
    return it->second && it->second->valid();
}

void Middleware::dispatcher_loop() {
    while (!shutdown_.load()) {
#ifdef __linux__
        // Collect subscriptions with their shared_ptrs (keeps them alive during dispatch)
        std::vector<std::pair<int, std::shared_ptr<detail::SubscriptionBase>>> ready_subs;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& [id, sub] : subscriptions_) {
                if (sub) {
                    int fd = sub->get_fd();
                    if (fd >= 0) {
                        ready_subs.emplace_back(fd, sub);  // shared_ptr copy
                    }
                }
            }
        }

        if (ready_subs.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Poll for events (outside lock, but shared_ptrs keep objects alive)
        std::vector<struct pollfd> pfds(ready_subs.size());
        for (std::size_t i = 0; i < ready_subs.size(); ++i) {
            pfds[i].fd = ready_subs[i].first;
            pfds[i].events = POLLIN;
            pfds[i].revents = 0;
        }

        int ret = ::poll(pfds.data(), static_cast<nfds_t>(pfds.size()), 10);  // 10ms timeout
        if (ret <= 0) {
            continue;
        }

        // Process ready subscriptions - shared_ptr keeps objects alive
        for (std::size_t i = 0; i < pfds.size(); ++i) {
            if (pfds[i].revents & POLLIN) {
                auto& sub = ready_subs[i].second;
                // Drain all messages - callback invoked without lock held
                while (sub->spin_once()) {
                    // Continue until no more messages
                }
            }
        }
#else
        // Non-Linux: simple polling with 1ms minimum latency
        std::vector<std::shared_ptr<detail::SubscriptionBase>> subs_to_process;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& [id, sub] : subscriptions_) {
                if (sub) {
                    subs_to_process.push_back(sub);  // shared_ptr copy
                }
            }
        }
        
        // Process outside the lock - shared_ptrs keep objects alive
        for (auto& sub : subs_to_process) {
            while (sub->spin_once()) {
                // Drain messages
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    }
}

void Middleware::ensure_dispatcher() {
    // Don't start dispatcher if already shutdown
    if (shutdown_.load() || dispatcher_running_.load()) {
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

bool Node::valid() const noexcept {
    return mw_ != nullptr;
}

} // namespace jr::mw

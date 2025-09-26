#include "middleware/middleware.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include "in_process_middleware.hpp"
#include "middleware/node.hpp"

namespace jr::mw {

void Middleware::publish(
  const std::string& topic,
  const google::protobuf::Message& message
) {
  const auto* desc = message.GetDescriptor();
  std::string type_name;
  if (desc) {
    auto sv = desc->full_name();
    type_name.assign(sv.data(), sv.size());
  }
  std::string payload;
  message.SerializeToString(&payload);
  do_publish_serialized(topic, type_name, std::move(payload));
}

void Middleware::publish_serialized(
  const std::string& topic,
  const std::string& type_full_name,
  const std::string& payload
) {
  do_publish_serialized(topic, type_full_name, payload);
}

Subscription Middleware::subscribe(
  const std::string& topic,
  const std::string& type_full_name,
  std::function<void(const google::protobuf::Message&)> callback
) {
  return do_subscribe_typed(topic, type_full_name, std::move(callback));
}

Subscription Middleware::subscribe_any(
  const std::string& topic,
  std::function<void(const std::string&, const google::protobuf::Message&)>
    callback
) {
  return do_subscribe_any(topic, std::move(callback));
}

std::shared_ptr<Middleware> Middleware::create(BackendKind backend) {
  switch (backend) {
  case BackendKind::InProcess:
    return std::make_shared<InProcessMiddleware>();
  default:
    return std::make_shared<InProcessMiddleware>();
  }
}

namespace {

struct RuntimeState {
  std::mutex mutex;
  std::condition_variable cv;
  std::atomic<bool> running{false};
  std::shared_ptr<Middleware> mw;
  BackendKind backend{BackendKind::InProcess};
  std::vector<std::shared_ptr<Node>> nodes;
  std::vector<std::thread> node_threads;
};

RuntimeState& runtime() {
  static RuntimeState s;
  return s;
}

} // namespace

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
  spin(std::vector{std::move(node)});
}

void spin(const std::vector<std::shared_ptr<Node>>& nodes) {
  auto& rt = runtime();
  std::unique_lock<std::mutex> lock(rt.mutex);
  
  // Stop any existing threads
  rt.running.store(false);
  for (auto& thread : rt.node_threads) {
    if (thread.joinable()) {
      lock.unlock();
      thread.join();
      lock.lock();
    }
  }
  rt.node_threads.clear();
  
  // Set up new nodes and start threads
  rt.nodes = nodes; // keep them alive while spinning
  rt.running.store(true);
  
  // Create a dedicated thread for each node
  for (const auto& node : nodes) {
    if (node) {
      rt.node_threads.emplace_back([&rt, node]() {
        // Each thread runs its own spin loop
        while (rt.running.load()) {
          node->spin_once();
          // Small delay to prevent busy waiting
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    }
  }

  rt.cv.wait(lock, [&]() { return !rt.running; });
  
  // Wait for all threads to complete
  lock.unlock();
  for (auto& thread : rt.node_threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  
  // Clear threads after joining
  lock.lock();
  rt.node_threads.clear();
}

void shutdown() {
  auto& rt = runtime();
  std::unique_lock<std::mutex> lock(rt.mutex);
  
  // Stop all threads
  rt.running.store(false);
  
  if (rt.mw) {
    rt.mw->shutdown();
    rt.mw.reset();
  }
  
  rt.nodes.clear();
  lock.unlock();
  
  rt.cv.notify_all();
}

} // namespace jr::mw
#pragma once

#include <middleware/backend.hpp>
#include <middleware_rs/fwd.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace jr::mw::detail {

struct TransportRecvResult {
  bool has_message;
  std::vector<std::uint8_t> data;
  bool closed;
};

class PublisherHandle {
public:
  virtual ~PublisherHandle() = default;
  virtual bool publish(const void* data, std::size_t len) = 0;
  virtual bool valid() const = 0;
};

class SubscriberHandle {
public:
  virtual ~SubscriberHandle() = default;
  virtual bool valid() const = 0;
  virtual int get_fd() const = 0;
  virtual TransportRecvResult recv() = 0;
};

std::shared_ptr<PublisherHandle> create_publisher_handle(
  const std::string& topic,
  Qos qos,
  std::size_t capacity
);

bool publish_handle(
  const std::shared_ptr<PublisherHandle>& handle,
  const void* data,
  std::size_t len
);

bool publisher_valid(const std::shared_ptr<PublisherHandle>& handle);

std::shared_ptr<SubscriberHandle> create_subscriber_handle(
  const std::string& topic,
  Qos qos,
  std::size_t capacity
);

bool subscriber_valid(const std::shared_ptr<SubscriberHandle>& handle);
int subscriber_fd(const std::shared_ptr<SubscriberHandle>& handle);
TransportRecvResult subscriber_recv(
  const std::shared_ptr<SubscriberHandle>& handle
);

bool init_backend();
bool set_backend_impl(Backend backend);
Backend get_backend_impl();
const char* backend_name_impl(Backend backend);

bool topic_exists_impl(const std::string& topic);
std::size_t subscriber_count_impl(const std::string& topic);

} // namespace jr::mw::detail

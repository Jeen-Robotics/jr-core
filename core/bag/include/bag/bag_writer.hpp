#pragma once

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

namespace jr::mw {

class Middleware;

class BagWriter {
public:
  explicit BagWriter(const std::string& path);
  explicit BagWriter(const std::string& path, std::shared_ptr<Middleware> mw);
  ~BagWriter();

  bool open();
  void close();

  // Record all messages published on a given topic
  void record_topic(const std::string& topic);

  // Record all topics (wildcard). Requires dynamic subscribe_any support.
  void record_all();

private:
  std::string path_;
  std::shared_ptr<Middleware> mw_;
  std::ofstream out_;
  std::mutex out_mutex_;
  std::vector<class Subscription> active_subs_;

  void write_record(
    const std::string& topic,
    const std::string& type_full_name,
    const std::string& payload,
    std::uint64_t ts_ns
  );
};

} // namespace jr::mw

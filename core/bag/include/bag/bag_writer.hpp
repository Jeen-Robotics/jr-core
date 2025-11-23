#pragma once

#include <bag/video_encoder_config.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace google {
namespace protobuf {
class Message;
}
} // namespace google

namespace jr::mw {

class Middleware;
class VideoEncoder;

class BagWriter {
public:
  explicit BagWriter(const std::string& path);
  explicit BagWriter(const std::string& path, std::shared_ptr<Middleware> mw);
  ~BagWriter();

  bool open();
  void close();

  // Set video encoder configuration
  void set_video_config(VideoEncoderConfig config);

  // Record all messages published on a given topic
  void record_topic(const std::string& topic);

  // Record all topics (wildcard). Requires dynamic subscribe_any support.
  void record_all();

  // Stop recording
  void stop();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;

  std::string path_;
  std::shared_ptr<Middleware> mw_;
  std::mutex out_mutex_;
  std::vector<class Subscription> active_subs_;

  bool is_open_ = false;

  // Video encoding state
  VideoEncoderConfig video_config_;
  std::map<std::string, std::unique_ptr<VideoEncoder>> video_encoders_;

  void write_record(
    const std::string& topic,
    const google::protobuf::Message& msg,
    std::uint64_t ts_ns
  );

  std::string get_video_path_for_topic(const std::string& topic) const;
};

} // namespace jr::mw

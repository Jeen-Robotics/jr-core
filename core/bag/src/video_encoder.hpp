#pragma once

#include <bag/video_encoder_config.hpp>

#include <sensor_msgs.pb.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace cv {
class VideoWriter;
}

namespace jr::mw {

// Reference to a frame in the video file
struct VideoFrameRef {
  std::string topic;
  std::string video_file; // Relative path to video file
  std::uint64_t frame_index;
  std::uint32_t width;
  std::uint32_t height;
  std::string encoding;
  std::uint32_t seq;
};

// Continuously encodes sensor_msgs::Image messages to a video file
class VideoEncoder {
public:
  // video_path: full path to output video file (e.g., "/path/to/bag_topic.avi")
  explicit VideoEncoder(
    const std::string& topic,
    const std::string& video_path,
    const VideoEncoderConfig& config = VideoEncoderConfig{}
  );
  ~VideoEncoder();

  // Add a frame to the video file
  // Returns frame reference if successful, nullopt otherwise
  std::optional<VideoFrameRef> add_frame(const sensor_msgs::Image& msg);

  // Get the relative video filename (for storing in bag)
  std::string get_video_filename() const;

  // Get current frame count
  std::uint64_t get_frame_count() const {
    return frame_count_;
  }

private:
  std::string topic_;
  std::string video_path_;
  std::string video_filename_; // Just the filename, not full path
  VideoEncoderConfig config_;

  // Video writer state
  std::unique_ptr<cv::VideoWriter> writer_;
  bool is_initialized_ = false;

  // Current video properties
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  std::string encoding_;
  std::uint64_t frame_count_ = 0;

  void init_writer(const sensor_msgs::Image& first_frame);
  int get_fourcc() const;
};

} // namespace jr::mw

#pragma once

#include <sensor_msgs.pb.h>

#include <map>
#include <memory>
#include <string>

namespace cv {
class VideoCapture;
}

namespace jr::mw {

// Manages video files and decodes specific frames
class VideoDecoder {
public:
  VideoDecoder();
  ~VideoDecoder();

  // Open a video file for reading
  // video_path: full path to the video file
  bool open_video_file(const std::string& video_path);

  // Decode a specific frame from an open video file
  // Returns the decoded frame as sensor_msgs::Image
  // video_path: which video file to read from
  // frame_index: which frame to decode
  // width, height, encoding: expected properties
  // seq: sequence number to set in the message
  bool decode_frame(
    const std::string& video_path,
    std::uint64_t frame_index,
    std::uint32_t width,
    std::uint32_t height,
    const std::string& encoding,
    std::uint32_t seq,
    sensor_msgs::Image& msg
  );

private:
  // Cache of open video files
  struct VideoFile {
    std::unique_ptr<cv::VideoCapture> capture;
    std::uint64_t last_frame_index = 0;
  };

  std::map<std::string, VideoFile> video_files_;

  // Get or open video file
  cv::VideoCapture* get_video_capture(const std::string& video_path);
};

} // namespace jr::mw

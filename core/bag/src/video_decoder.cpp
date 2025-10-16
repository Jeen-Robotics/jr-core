#include "video_decoder.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>

namespace jr::mw {

namespace {

sensor_msgs::Image to_msg(
  const cv::Mat& frame,
  std::uint32_t width,
  std::uint32_t height,
  const std::string& encoding,
  const std::uint32_t seq
) {
  sensor_msgs::Image msg;

  auto* header = msg.mutable_header();
  header->set_frame_id("camera");
  header->set_seq(seq);

  msg.set_encoding(encoding);
  msg.set_is_bigendian(false);
  msg.set_width(frame.cols);
  msg.set_height(frame.rows);
  msg.set_step(frame.step);
  msg.set_data(frame.data, frame.total() * frame.elemSize());

  return msg;
}

} // namespace

VideoDecoder::VideoDecoder() = default;

VideoDecoder::~VideoDecoder() {
  // Close all video files
  video_files_.clear();
}

bool VideoDecoder::open_video_file(const std::string& video_path) {
  // Check if already open
  if (video_files_.find(video_path) != video_files_.end()) {
    return true;
  }

  VideoFile vf;
  vf.capture = std::make_unique<cv::VideoCapture>(video_path);

  if (!vf.capture->isOpened()) {
    std::cerr << "VideoDecoder: failed to open video file: " << video_path
              << "\n";
    return false;
  }

  video_files_[video_path] = std::move(vf);
  return true;
}

cv::VideoCapture* VideoDecoder::get_video_capture(const std::string& video_path
) {
  if (!open_video_file(video_path)) {
    return nullptr;
  }

  return video_files_[video_path].capture.get();
}

bool VideoDecoder::decode_frame(
  const std::string& video_path,
  const std::uint64_t frame_index,
  const std::uint32_t width,
  const std::uint32_t height,
  const std::string& encoding,
  const std::uint32_t seq,
  sensor_msgs::Image& msg
) {
  cv::VideoCapture* capture = get_video_capture(video_path);
  if (!capture) {
    return false;
  }

  auto& vf = video_files_[video_path];

  // Seek to frame if needed (optimization: skip if sequential)
  if (frame_index != vf.last_frame_index) {
    capture->set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frame_index));
  }

  cv::Mat frame;
  if (!capture->read(frame)) {
    std::cerr << "VideoDecoder: failed to read frame " << frame_index
              << " from " << video_path << "\n";
    return false;
  }

  if (frame.empty()) {
    std::cerr << "VideoDecoder: empty frame " << frame_index << " from "
              << video_path << "\n";
    return false;
  }

  // Convert BGR back to grayscale if needed
  if (encoding == "8UC1" && frame.channels() == 3) {
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
  }

  // Convert to message
  msg = to_msg(frame, width, height, encoding, seq);

  vf.last_frame_index = frame_index + 1;
  return true;
}

} // namespace jr::mw

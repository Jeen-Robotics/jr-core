#include "video_encoder.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

namespace jr::mw {

namespace {

int deserialize_encoding(const std::string& encoding) {
  if (encoding == "8UC1")
    return CV_8UC1;
  if (encoding == "8UC3")
    return CV_8UC3;
  if (encoding == "8UC4")
    return CV_8UC4;
  return -1;
}

cv::Mat from_msg(const sensor_msgs::Image& msg) {
  return cv::Mat(
           msg.height(),
           msg.width(),
           deserialize_encoding(msg.encoding()),
           const_cast<char*>(msg.data().data()),
           msg.step()
  )
    .clone();
}

} // namespace

VideoEncoder::VideoEncoder(
  const std::string& topic,
  const std::string& video_path,
  const VideoEncoderConfig& config
)
    : topic_(topic)
    , video_path_(video_path)
    , config_(config) {
  // Extract just the filename from the full path
  video_filename_ = std::filesystem::path(video_path).filename().string();
}

VideoEncoder::~VideoEncoder() {
  if (writer_ && writer_->isOpened()) {
    writer_->release();
  }
}

int VideoEncoder::get_fourcc() const {
  switch (config_.codec) {
  case VideoEncoderConfig::Codec::H264:
    return cv::VideoWriter::fourcc('H', '2', '6', '4');
  case VideoEncoderConfig::Codec::H265:
    return cv::VideoWriter::fourcc('H', 'E', 'V', 'C');
  case VideoEncoderConfig::Codec::MJPEG:
    return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  default:
    return cv::VideoWriter::fourcc('H', '2', '6', '4');
  }
}

void VideoEncoder::init_writer(const sensor_msgs::Image& first_frame) {
  width_ = first_frame.width();
  height_ = first_frame.height();
  encoding_ = first_frame.encoding();

  int fourcc = get_fourcc();
  writer_ = std::make_unique<cv::VideoWriter>(
    video_path_,
    fourcc,
    config_.fps,
    cv::Size(width_, height_),
    encoding_ != "8UC1" // isColor
  );

  if (!writer_->isOpened()) {
    std::cerr << "VideoEncoder: failed to open video writer for topic "
              << topic_ << " at " << video_path_ << "\n";
    writer_.reset();
    return;
  }

  is_initialized_ = true;
  std::cout << "VideoEncoder: writing video to " << video_path_ << "\n";
}

std::string VideoEncoder::get_video_filename() const {
  return video_filename_;
}

std::optional<VideoFrameRef> VideoEncoder::add_frame(
  const sensor_msgs::Image& msg
) {
  // Initialize on first frame
  if (!is_initialized_) {
    init_writer(msg);
    if (!is_initialized_) {
      return std::nullopt;
    }
  }

  // Check if frame dimensions match
  if (msg.width() != width_ || msg.height() != height_ ||
      msg.encoding() != encoding_) {
    std::cerr << "VideoEncoder: frame dimensions or encoding changed for "
              << topic_ << ", cannot continue encoding\n";
    return std::nullopt;
  }

  // Convert a message to cv::Mat
  const auto frame = from_msg(msg);

  // Write frame immediately to a video file
  writer_->write(frame);

  // Create frame reference
  VideoFrameRef ref;
  ref.topic = topic_;
  ref.video_file = video_filename_;
  ref.frame_index = frame_count_;
  ref.width = width_;
  ref.height = height_;
  ref.encoding = encoding_;
  ref.seq = msg.header().seq();

  frame_count_++;

  return ref;
}

} // namespace jr::mw

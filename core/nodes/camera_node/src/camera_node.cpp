#include "camera_node/camera_node.hpp"

#include <opencv2/videoio.hpp>

#include <thread>

namespace jr {

namespace {

std::string serialize(const int encoding) {
  switch (encoding) {
  case CV_8UC1:
    return "8UC1";
  case CV_8UC3:
    return "8UC3";
  default:
    return "unknown";
  }
}

sensor_msgs::Image to_msg(const cv::Mat& frame) {
  const auto now_sec = std::chrono::time_point_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now()
                       )
                         .time_since_epoch()
                         .count() *
                       1e-9;

  sensor_msgs::Image msg;

  auto* header = msg.mutable_header();
  header->mutable_stamp()->set_sec(static_cast<int32_t>(now_sec));
  header->mutable_stamp()->set_nsec(static_cast<int32_t>(now_sec * 1e9));
  header->set_frame_id("camera");
  header->set_seq(0);

  msg.set_encoding(serialize(frame.type()));
  msg.set_is_bigendian(false);
  msg.set_width(frame.cols);
  msg.set_height(frame.rows);
  msg.set_step(frame.step);
  msg.set_data(frame.data, frame.total() * frame.elemSize());

  return msg;
}

} // namespace

class CameraNode::Impl {
public:
  explicit Impl(const int camera_idx)
      : cap_(camera_idx) {
    cap_.set(cv::CAP_PROP_FOURCC, CV_FOURCC_MACRO('B', 'G', 'R', '3'));
  }

  bool valid() const {
    return cap_.isOpened();
  }

  void spin_once(const mw::Publisher<sensor_msgs::Image>& pub) {
    if (!valid()) {
      return;
    }

    cv::Mat frame;
    cap_ >> frame;
    if (!frame.empty()) {
      pub.publish(to_msg(frame));
    }
  }

private:
  cv::VideoCapture cap_;
};

CameraNode::CameraNode(const std::string& node_name, const Config& config)
    : CameraNode(node_name, config, mw::get()) {
}

CameraNode::CameraNode(
  const std::string& node_name,
  const Config& config,
  const std::shared_ptr<mw::Middleware>& mw
)
    : Node(node_name, mw)
    , pub_(create_publisher<sensor_msgs::Image>(config.topic_name))
    , pimpl_(std::make_unique<Impl>(config.camera_idx)) {
}

CameraNode::~CameraNode() = default;

void CameraNode::spin_once() {
  pimpl_->spin_once(pub_);
  // TODO: add fps or delay param to config
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool CameraNode::valid() const noexcept {
  return Node::valid() && pimpl_->valid();
}

} // namespace jr
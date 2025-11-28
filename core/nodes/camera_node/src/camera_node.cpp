#include "camera_node/camera_node.hpp"
#include "calibration_extractor.hpp"

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

sensor_msgs::Image to_msg(const cv::Mat& frame, const uint64_t now_ns) {
  const auto now_sec = static_cast<double>(now_ns) * 1e-9;

  sensor_msgs::Image msg;

  auto* header = msg.mutable_header();
  header->mutable_stamp()->set_sec(static_cast<int32_t>(now_sec));
  header->mutable_stamp()->set_nsec(static_cast<int32_t>(now_ns % 1000000000));
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
      : cap_(camera_idx)
      , extractor_(camera_idx) {
    cap_.set(cv::CAP_PROP_FOURCC, CV_FOURCC_MACRO('B', 'G', 'R', '3'));
  }

  bool valid() const {
    return cap_.isOpened();
  }

  void spin_once(
    const mw::Publisher<sensor_msgs::Image>& pub,
    const mw::Publisher<sensor_msgs::CameraInfo>& info_pub,
    const mw::Publisher<geometry_msgs::PoseStamped>& pose_pub
  ) {
    if (!valid()) {
      return;
    }

    if (cap_.grab()) {
      auto now = std::chrono::system_clock::now();
      auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      now.time_since_epoch()
      )
                      .count();

      cv::Mat frame;
      if (cap_.retrieve(frame) && !frame.empty()) {
        double msec = cap_.get(cv::CAP_PROP_POS_MSEC);

        // Heuristic: If timestamp is > Year 2000 (946684800000 ms),
        // assume it is a valid system epoch timestamp.
        if (msec > 946684800000.0) {
          now_ns = static_cast<uint64_t>(msec * 1e6);
        }

        auto img_msg = to_msg(frame, now_ns);
        pub.publish(img_msg);

        if (!cached_info_ && !extraction_attempted_) {
          cached_info_ = extractor_.extract(frame.cols, frame.rows);
          cached_pose_ = extractor_.extract_pose();
          extraction_attempted_ = true;
        }

        if (cached_info_) {
          auto info_msg = *cached_info_;
          *info_msg.mutable_header() = img_msg.header();
          info_pub.publish(info_msg);
        }

        if (cached_pose_) {
          auto pose_msg = *cached_pose_;
          *pose_msg.mutable_header() = img_msg.header();
          pose_pub.publish(pose_msg);
        }
      }
    }
  }

private:
  cv::VideoCapture cap_;
  CalibrationExtractor extractor_;
  std::optional<sensor_msgs::CameraInfo> cached_info_;
  std::optional<geometry_msgs::PoseStamped> cached_pose_;
  bool extraction_attempted_ = false;
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
    , config_(config)
    , pub_(create_publisher<sensor_msgs::Image>(config.topic_name))
    , info_pub_(
        create_publisher<sensor_msgs::CameraInfo>(config.topic_name + "/info")
      )
    , pose_pub_(create_publisher<geometry_msgs::PoseStamped>(
        config.topic_name + "/pose"
      ))
    , pimpl_(std::make_unique<Impl>(config.camera_idx)) {
}

CameraNode::~CameraNode() = default;

void CameraNode::spin_once() {
  pimpl_->spin_once(pub_, info_pub_, pose_pub_);
  std::this_thread::sleep_for(std::chrono::milliseconds(config_.delay_ms));
}

bool CameraNode::valid() const noexcept {
  return Node::valid() && pimpl_->valid();
}

} // namespace jr

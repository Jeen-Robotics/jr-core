#include <bag/bag_reader.hpp>
#include <bag/bag_writer.hpp>
#include <camera_node/camera_node.hpp>
#include <middleware/middleware.hpp>
#include <middleware/node.hpp>

#include <sensor_msgs.pb.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>
#include <thread>

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

int deserialize(const std::string& encoding) {
  if (encoding == "8UC1")
    return CV_8UC1;
  if (encoding == "8UC3")
    return CV_8UC3;
  return -1;
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

cv::Mat from_msg(const sensor_msgs::Image& msg) {
  return cv::Mat(
           msg.height(),
           msg.width(),
           deserialize(msg.encoding()),
           const_cast<char*>(msg.data().data()),
           msg.step()
  )
    .clone();
}

class GraySubscriberNode final : public jr::mw::Node {
public:
  explicit GraySubscriberNode()
      : jr::mw::Node("gray_subscriber")
      , sub_(
          create_subscription<sensor_msgs::Image>(
            "/camera/frame",
            [this](const sensor_msgs::Image& frame) { on_frame(frame); }
          )
        )
      , pub_(create_publisher<sensor_msgs::Image>("/camera/gray")) {
  }

private:
  jr::mw::Subscription sub_;
  jr::mw::Publisher<sensor_msgs::Image> pub_;

  void on_frame(const sensor_msgs::Image& frame) const {
    cv::Mat gray = from_msg(frame);
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    pub_.publish(to_msg(gray));
  }
};

class VisualizerNode final : public jr::mw::Node {
public:
  explicit VisualizerNode()
      : jr::mw::Node("visualizer") {
    sub_frame_ = create_subscription<sensor_msgs::Image>(
      "/camera/frame",
      [this](const sensor_msgs::Image& frame) { on_frame(frame); }
    );
    sub_gray_ = create_subscription<sensor_msgs::Image>(
      "/camera/gray",
      [this](const sensor_msgs::Image& gray) { on_gray(gray); }
    );
  }

  void on_frame(const sensor_msgs::Image& frame) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_ = from_msg(frame);
  }

  void on_gray(const sensor_msgs::Image& gray) {
    std::lock_guard<std::mutex> lock(gray_mutex_);
    gray_ = from_msg(gray);
  }

  void spin_once() override {
    cv::Mat frame_copy;
    cv::Mat gray_copy;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!frame_.empty())
        frame_copy = frame_.clone();
    }
    {
      std::lock_guard<std::mutex> lock(gray_mutex_);
      if (!gray_.empty())
        gray_copy = gray_.clone();
    }

    if (!frame_copy.empty()) {
      cv::imshow("Frame", frame_copy);
    }
    if (!gray_copy.empty()) {
      cv::imshow("Gray", gray_copy);
    }

    const auto key = cv::waitKey(1);
    if (key == 'q') {
      jr::mw::shutdown();
    }
  }

private:
  cv::Mat frame_;
  cv::Mat gray_;

  std::mutex frame_mutex_;
  std::mutex gray_mutex_;

  jr::mw::Subscription sub_frame_;
  jr::mw::Subscription sub_gray_;
};

#define WRITE_MODE 1
int main() {
  jr::mw::init();

  const auto vis_node = std::make_shared<VisualizerNode>();
#if WRITE_MODE
  const auto pub_node = std::make_shared<jr::CameraNode>(
    "camera_publisher",
    jr::CameraNode::Config{"/camera/frame", 0}
  );
  const auto sub_node = std::make_shared<GraySubscriberNode>();
#endif

#if WRITE_MODE
  jr::mw::BagWriter bag_writer("camera_test.bag");

  // Optional: Configure video encoding parameters
  // jr::mw::VideoEncoderConfig video_config;
  // video_config.codec = jr::mw::VideoEncoderConfig::Codec::H264;
  // video_config.fps = 30.0;
  // bag_writer.set_video_config(video_config);
#else
  jr::mw::BagReader bag_reader("camera_test.bag");
#endif

  // TODO:
  // - add spin_async?
  std::thread spin_thread([&]() {
#if WRITE_MODE
    jr::mw::spin(
      std::vector<std::shared_ptr<jr::mw::Node>>{
        pub_node,
        sub_node,
      }
    );
#else
    bag_reader.play();
    jr::mw::shutdown();
#endif
  });

#if WRITE_MODE
  bag_writer.record_all();
#endif

  jr::mw::spin(vis_node);

#if !WRITE_MODE
  bag_reader.stop();
#endif

  spin_thread.join();

  return 0;
}
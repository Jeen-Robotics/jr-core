#include <middleware/middleware.hpp>
#include <middleware/node.hpp>

#include <sensor_msgs.pb.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>
#include <thread>

std::string serialize(int encoding) {
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
  auto now_sec = std::chrono::time_point_cast<std::chrono::nanoseconds>(
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

class CameraPublisherNode : public jr::mw::Node {
public:
  explicit CameraPublisherNode()
      : jr::mw::Node("camera_publisher")
      , camera_(0)
      , pub_(create_publisher<sensor_msgs::Image>("/camera/frame")) {
  }

  void spin_once() override {
    if (!camera_.isOpened())
      return;

    cv::Mat frame;
    camera_ >> frame;
    if (frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      return;
    }

    pub_.publish(to_msg(frame));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

private:
  cv::VideoCapture camera_;
  jr::mw::Publisher<sensor_msgs::Image> pub_;
};

class GraySubscriberNode : public jr::mw::Node {
public:
  explicit GraySubscriberNode()
      : jr::mw::Node("gray_subscriber")
      , sub_(create_subscription<sensor_msgs::Image>(
          "/camera/frame",
          [this](const sensor_msgs::Image& frame) { on_frame(frame); }
        ))
      , pub_(create_publisher<sensor_msgs::Image>("/camera/gray")) {
  }

private:
  jr::mw::Subscription sub_;
  jr::mw::Publisher<sensor_msgs::Image> pub_;

  void on_frame(const sensor_msgs::Image& frame) {
    cv::Mat gray = from_msg(frame);
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    pub_.publish(to_msg(gray));
  }
};

class VisualizerNode : public jr::mw::Node {
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

    int key = cv::waitKey(1);
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

int main() {
  jr::mw::init();

  auto pub_node = std::make_shared<CameraPublisherNode>();
  auto sub_node = std::make_shared<GraySubscriberNode>();
  auto vis_node = std::make_shared<VisualizerNode>();

  // TODO:
  // - add spin_async?
  // - add bag support
  // - write sample bag
  std::thread spin_thread([&]() {
    jr::mw::spin(std::vector<std::shared_ptr<jr::mw::Node>>{pub_node, sub_node}
    );
  });

  jr::mw::spin(vis_node);

  spin_thread.join();

  return 0;
}
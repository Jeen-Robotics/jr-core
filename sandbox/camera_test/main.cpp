#include <atomic>
#include <middleware/middleware.hpp>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

class CameraPublisherNode : public jr::mw::Node {
public:
  explicit CameraPublisherNode(cv::Mat& frame, std::mutex& frame_mutex)
      : jr::mw::Node("camera_publisher")
      , pub_(create_publisher<cv::Mat>("/camera/frame"))
      , running_(true)
      , frame_(frame)
      , frame_mutex_(frame_mutex)
      , worker_([this]() { run_loop(); }) {
  }

  ~CameraPublisherNode() override {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void run_loop() {
    cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
      return;
    }
    for (;;) {
      if (!running_.load())
        break;
      cv::Mat local;
      camera >> local;
      if (local.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_ = local;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      pub_.publish(local);
    }
  }

  jr::mw::Publisher<cv::Mat> pub_;
  std::atomic<bool> running_;
  cv::Mat& frame_;
  std::mutex& frame_mutex_;
  std::thread worker_;
};

class GraySubscriberNode : public jr::mw::Node {
public:
  explicit GraySubscriberNode(cv::Mat& gray, std::mutex& gray_mutex)
      : jr::mw::Node("gray_subscriber")
      , gray_(gray)
      , gray_mutex_(gray_mutex)
      , sub_(create_subscription<cv::Mat>(
          "/camera/frame",
          [this](const cv::Mat& frame) { on_frame(frame); }
        )) {
  }

private:
  std::mutex& gray_mutex_;
  cv::Mat& gray_;
  jr::mw::Subscription sub_;

  void on_frame(const cv::Mat& frame) {
    cv::Mat local_gray;
    cv::cvtColor(frame, local_gray, cv::COLOR_BGR2GRAY);
    {
      std::lock_guard<std::mutex> lock(gray_mutex_);
      gray_ = std::move(local_gray);
    }
  }
};

int main() {
  cv::Mat frame;
  cv::Mat gray;

  jr::mw::init();

  std::mutex frame_mutex;
  std::mutex gray_mutex;

  auto pub_node = std::make_shared<CameraPublisherNode>(frame, frame_mutex);
  auto sub_node = std::make_shared<GraySubscriberNode>(gray, gray_mutex);

  std::thread spin_thread([&]() {
    jr::mw::spin(std::vector<std::shared_ptr<jr::mw::Node>>{pub_node, sub_node}
    );
  });

  for (;;) {
    cv::Mat frame_copy;
    cv::Mat gray_copy;
    {
      std::lock_guard<std::mutex> lock(frame_mutex);
      if (!frame.empty())
        frame_copy = frame.clone();
    }
    {
      std::lock_guard<std::mutex> lock(gray_mutex);
      if (!gray.empty())
        gray_copy = gray.clone();
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
      break;
    }
  }

  spin_thread.join();

  return 0;
}
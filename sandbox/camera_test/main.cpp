#include <middleware/middleware.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main() {
  auto mw = jr::mw::get();

  std::mutex gray_mutex;
  cv::Mat gray;

  auto sub = mw->subscribe<cv::Mat>(
    "/camera/frame",
    [&gray, &gray_mutex](const cv::Mat& frame) {
      std::lock_guard<std::mutex> lock(gray_mutex);
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
  );

  cv::VideoCapture camera(0);

  while (true) {
    cv::Mat frame;
    camera >> frame;
    if (frame.empty()) {
      break;
    }

    mw->publish<cv::Mat>("/camera/frame", frame);

    cv::imshow("Frame", frame);
    auto key = cv::waitKey(1);
    if (key == 'q') {
      break;
    }

    {
      std::lock_guard<std::mutex> lock(gray_mutex);
      if (!gray.empty()) {
        cv::imshow("Gray", gray);
      }
    }
  }

  return 0;
}
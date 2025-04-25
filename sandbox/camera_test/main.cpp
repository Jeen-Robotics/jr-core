#include <opencv2/highgui.hpp>

int main() {
  cv::VideoCapture camera(0);

  while (true) {
    cv::Mat frame;
    camera >> frame;
    if (frame.empty()) {
      break;
    }

    cv::imshow("Frame", frame);
    auto key = cv::waitKey(1);
    if (key == 'q') {
      break;
    }
  }

  return 0;
}
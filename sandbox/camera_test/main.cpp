#include <camera/camera_processor.h>
#include <opencv2/highgui.hpp>

int main() {
  CameraProcessor camera;
  if (!camera.initialize()) {
    return 1;
  }

  while (true) {
    cv::Mat frame = camera.processFrame(640, 480);
    if (frame.empty()) {
      break;
    }

    cv::imshow("Frame", frame);
    auto key = cv::waitKey(1);
    if (key == 'q') {
      camera.stop();
      break;
    }
  }

  return 0;
}
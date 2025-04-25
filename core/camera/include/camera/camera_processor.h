#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

class CameraProcessor {
public:
  CameraProcessor();
  ~CameraProcessor();

  bool initialize(int cameraIndex = 0);
  bool isInitialized() const;
  void stop();

  // Process a single frame and return the grayscale image data
  cv::Mat processFrame(int width, int height);

private:
  std::unique_ptr<cv::VideoCapture> videoCapture_;
  bool isInitialized_;
};
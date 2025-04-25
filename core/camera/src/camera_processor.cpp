#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <camera/camera_processor.h>

CameraProcessor::CameraProcessor() : isInitialized_(false) {}

CameraProcessor::~CameraProcessor() { stop(); }

bool CameraProcessor::initialize(int cameraIndex) {
  if (isInitialized_) {
    return true;
  }

  videoCapture_ = std::make_unique<cv::VideoCapture>(cameraIndex);
  if (!videoCapture_->isOpened()) {
    return false;
  }

  isInitialized_ = true;
  return true;
}

bool CameraProcessor::isInitialized() const { return isInitialized_; }

void CameraProcessor::stop() {
  if (videoCapture_) {
    videoCapture_->release();
  }
  isInitialized_ = false;
}

cv::Mat CameraProcessor::processFrame(int width, int height) {
  if (!isInitialized_ || !videoCapture_) {
    return cv::Mat();
  }

  cv::Mat frame;
  if (!videoCapture_->read(frame)) {
    return cv::Mat();
  }

  // Convert to grayscale and detect edges
  cv::Mat grayFrame, edges;
  cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 1.5);
  cv::Canny(grayFrame, edges, 100, 200);
  grayFrame = edges;

  // Resize to a reasonable size
  cv::resize(grayFrame, grayFrame, cv::Size(width, height));

  return grayFrame;
}
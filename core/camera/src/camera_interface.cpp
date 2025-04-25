#include <memory>

#include <camera/camera_interface.h>
#include <camera/camera_processor.h>

namespace {
std::unique_ptr<CameraProcessor> cameraProcessor;
cv::Mat currentFrame;
} // namespace

void initializeCamera() {
  if (!cameraProcessor) {
    cameraProcessor = std::make_unique<CameraProcessor>();
    cameraProcessor->initialize(0);
  }
}

bool isCameraInitialized() {
  return cameraProcessor && cameraProcessor->isInitialized();
}

void stopCamera() {
  if (cameraProcessor) {
    cameraProcessor->stop();
    cameraProcessor.reset();
  }
}

uint8_t *processFrame(int width, int height) {
  if (!cameraProcessor || !cameraProcessor->isInitialized()) {
    return nullptr;
  }

  currentFrame = cameraProcessor->processFrame(width, height);
  if (currentFrame.empty()) {
    return nullptr;
  }

  cv::Mat rgba;
  cv::cvtColor(currentFrame, rgba, cv::COLOR_GRAY2BGRA);

  return rgba.data;
}
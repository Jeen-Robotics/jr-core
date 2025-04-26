#pragma once

#include <functional>
#include <memory>

namespace jr {

class AndroidCameraStreamer {
public:
  AndroidCameraStreamer();
  ~AndroidCameraStreamer();

  // Initialize the camera with the given parameters
  bool initialize(int32_t width, int32_t height, int32_t format = 1);

  // Start streaming images
  bool startStreaming();

  // Stop streaming images
  void stopStreaming();

  // Set callback for receiving image frames
  void setFrameCallback(std::function<void(const uint8_t *, size_t)> callback);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace jr
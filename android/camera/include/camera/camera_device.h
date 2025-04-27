#pragma once

#include <functional>
#include <memory>

namespace jr::android
{

class CameraDevice
{
public:
  CameraDevice();
  ~CameraDevice();

  // Get the number of cameras available
  int getNumberOfCameras() const;

  // Initialize the camera with the given parameters
  bool open(int width, int height, int camera_idx);

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

} // namespace jr::android
#pragma once

#include <functional>
#include <memory>

#include <jr_imgproc/image.h>

namespace jr::android {

using FrameCallback = std::function<void(const jr_planar_image_t&)>;
using VoidCallback = std::function<void()>;

class CameraDevice {
public:
  CameraDevice();
  ~CameraDevice();

  // Get the number of cameras available
  int getNumberOfCameras() const;

  // Initialize the camera with the given parameters
  bool open(int width, int height, int camera_idx);

  // Close the camera
  void close();

  // Start streaming images
  bool startStreaming();

  // Stop streaming images
  void stopStreaming();

  // Set callback for receiving image frames
  void setFrameCallback(FrameCallback callback);

  // Set callback on session state change
  void setSessionReadyCallback(VoidCallback callback);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace jr::android
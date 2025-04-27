#include "camera/camera_device_api.h"

#include <functional>
#include <memory>

#if defined(ANDROID)
#include "camera/camera_device.h"
#else
class DummyCameraDevice {
  public:
    int getNumberOfCameras() const { return 0; }
    bool open(int width, int height, int camera_idx) { return false; }
    bool startStreaming() { return false; }
    void stopStreaming() {}
    void setFrameCallback(std::function<void(const uint8_t*, size_t)> callback) {}
};
#endif

struct camera_device
{
  // TODO: Create camera device interface and factory
#if defined(ANDROID)
  std::unique_ptr<jr::android::CameraDevice> impl;
#else
  std::unique_ptr<DummyCameraDevice> impl;
#endif
};

camera_device_t* camera_device_create() {
  auto* device = new camera_device_t;
#if defined(ANDROID)
  device->impl = std::make_unique<jr::android::CameraDevice>();
#else
  device->impl = std::make_unique<DummyCameraDevice>();
#endif
  return device;
}

void camera_device_destroy(camera_device_t* device) {
  if (device) {
    delete device;
  }
}

int camera_device_get_number_of_cameras(camera_device_t* device) {
  if (!device || !device->impl) {
    return 0;
  }
  return device->impl->getNumberOfCameras();
}

bool camera_device_open(
  camera_device_t* device,
  int32_t width,
  int32_t height,
  int32_t camera_idx
) {
  if (!device || !device->impl) {
    return false;
  }
  return device->impl->open(width, height, camera_idx);
}

bool camera_device_start_streaming(camera_device_t* device) {
  if (!device || !device->impl) {
    return false;
  }
  return device->impl->startStreaming();
}

void camera_device_stop_streaming(camera_device_t* device) {
  if (device && device->impl) {
    device->impl->stopStreaming();
  }
}

void camera_device_set_frame_callback(
  camera_device_t* device,
  camera_device_frame_callback_t callback,
  void* user_data
) {
  if (!device || !device->impl) {
    return;
  }

  device->impl->setFrameCallback(
    [callback, user_data](const uint8_t* data, size_t size) {
    if (callback) {
      callback(data, size, user_data);
    }
  });
}
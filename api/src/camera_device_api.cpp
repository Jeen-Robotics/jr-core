#include "jr_api/camera_device_api.h"

#include <memory>

#include <jr_imgproc/image.h>
#include <jr_imgproc/imgproc.h>

#if defined(ANDROID)
#include "jr_android/camera_device.hpp"
#else
#include <functional>

class DummyCameraDevice {
public:
  int getNumberOfCameras() const {
    return 0;
  }
  bool open(int width, int height, int camera_idx) {
    return false;
  }
  bool startStreaming() {
    return false;
  }
  void stopStreaming() {
  }
  void setFrameCallback(std::function<void(const Image*)> callback) {
  }
};
#endif

struct jr_camera_device {
  // TODO: Create camera device interface and factory
#if defined(ANDROID)
  std::unique_ptr<jr::android::CameraDevice> impl;
#else
  std::unique_ptr<DummyCameraDevice> impl;
#endif
};

jr_camera_device_t* jr_camera_device_create() {
  auto* device = new jr_camera_device_t;
#if defined(ANDROID)
  device->impl = std::make_unique<jr::android::CameraDevice>();
#else
  device->impl = std::make_unique<DummyCameraDevice>();
#endif
  return device;
}

void jr_camera_device_destroy(jr_camera_device_t* device) {
  if (device) {
    delete device;
  }
}

int jr_camera_device_get_number_of_cameras(jr_camera_device_t* device) {
  if (!device || !device->impl) {
    return 0;
  }
  return device->impl->getNumberOfCameras();
}

bool jr_camera_device_open(
  jr_camera_device_t* device,
  int32_t width,
  int32_t height,
  int32_t camera_idx
) {
  if (!device || !device->impl) {
    return false;
  }
  return device->impl->open(width, height, camera_idx);
}

bool jr_camera_device_start_streaming(jr_camera_device_t* device) {
  if (!device || !device->impl) {
    return false;
  }
  return device->impl->startStreaming();
}

void jr_camera_device_stop_streaming(jr_camera_device_t* device) {
  if (device && device->impl) {
    device->impl->stopStreaming();
  }
}

void jr_camera_device_set_frame_callback(
  jr_camera_device_t* device,
  jr_camera_device_frame_callback_t callback,
  void* user_data
) {
  if (!device || !device->impl) {
    return;
  }

  device->impl->setFrameCallback([callback,
                                  user_data](const jr_planar_image_t& image) {
    if (callback) {
      jr_image_t rgba_image;
      yuv2rgba(&image, &rgba_image);
      callback(&rgba_image, user_data);
      jr_image_destroy(&rgba_image);
    }
  });
}
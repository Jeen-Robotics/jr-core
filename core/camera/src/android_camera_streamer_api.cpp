#include "camera/android_camera_streamer_api.h"
#include "camera/android_camera_streamer.h"

struct jr_android_camera_streamer {
  std::unique_ptr<jr::AndroidCameraStreamer> impl;
};

jr_android_camera_streamer_t *jr_android_camera_streamer_create() {
  auto *streamer = new jr_android_camera_streamer_t;
  streamer->impl = std::make_unique<jr::AndroidCameraStreamer>();
  return streamer;
}

void jr_android_camera_streamer_destroy(
    jr_android_camera_streamer_t *streamer) {
  if (streamer) {
    delete streamer;
  }
}

int jr_android_camera_streamer_initialize(
    jr_android_camera_streamer_t *streamer, int32_t width, int32_t height,
    int32_t format) {
  if (!streamer || !streamer->impl) {
    return 0;
  }
  return streamer->impl->initialize(width, height, format) ? 1 : 0;
}

int jr_android_camera_streamer_start_streaming(
    jr_android_camera_streamer_t *streamer) {
  if (!streamer || !streamer->impl) {
    return 0;
  }
  return streamer->impl->startStreaming() ? 1 : 0;
}

void jr_android_camera_streamer_stop_streaming(
    jr_android_camera_streamer_t *streamer) {
  if (streamer && streamer->impl) {
    streamer->impl->stopStreaming();
  }
}

void jr_android_camera_streamer_set_frame_callback(
    jr_android_camera_streamer_t *streamer,
    jr_android_camera_frame_callback_t callback, void *user_data) {
  if (!streamer || !streamer->impl) {
    return;
  }

  streamer->impl->setFrameCallback(
      [callback, user_data](const uint8_t *data, size_t size) {
        if (callback) {
          callback(data, size, user_data);
        }
      });
}
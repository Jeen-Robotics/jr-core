#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to the camera device
typedef struct camera_device camera_device_t;

// Create a new camera device instance
camera_device_t* camera_device_create();

// Destroy an camera device instance
void camera_device_destroy(camera_device_t* device);

// Get the number of cameras available
int camera_device_get_number_of_cameras(camera_device_t* device);

// Open the camera with the given parameters
bool camera_device_open(
  camera_device_t* device,
  int32_t width,
  int32_t height,
  int32_t camera_idx
);

// Start streaming images
bool camera_device_start_streaming(camera_device_t* device);

// Stop streaming images
void camera_device_stop_streaming(camera_device_t* device);

// Callback type for receiving image frames
typedef void (*camera_device_frame_callback_t)(
  const uint8_t* data,
  size_t size,
  void* user_data
);

// Set callback for receiving image frames
void camera_device_set_frame_callback(
  camera_device_t* device,
  camera_device_frame_callback_t callback,
  void* user_data
);

#ifdef __cplusplus
}
#endif
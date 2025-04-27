#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <jr_imgproc/image.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to the camera device
typedef struct jr_camera_device jr_camera_device_t;

// Create a new camera device instance
jr_camera_device_t* jr_camera_device_create();

// Destroy an camera device instance
void jr_camera_device_destroy(jr_camera_device_t* device);

// Get the number of cameras available
int jr_camera_device_get_number_of_cameras(jr_camera_device_t* device);

// Open the camera with the given parameters
bool jr_camera_device_open(
  jr_camera_device_t* device,
  int32_t width,
  int32_t height,
  int32_t camera_idx
);

// Start streaming images
bool jr_camera_device_start_streaming(jr_camera_device_t* device);

// Stop streaming images
void jr_camera_device_stop_streaming(jr_camera_device_t* device);

// Callback type for receiving image frames
typedef void (*jr_camera_device_frame_callback_t)(
  const jr_image_t* image,
  void* user_data
);

// Set callback for receiving image frames
void jr_camera_device_set_frame_callback(
  jr_camera_device_t* device,
  jr_camera_device_frame_callback_t callback,
  void* user_data
);

#ifdef __cplusplus
}
#endif
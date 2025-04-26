#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to the Android camera streamer
typedef struct jr_android_camera_streamer jr_android_camera_streamer_t;

// Create a new Android camera streamer instance
jr_android_camera_streamer_t *jr_android_camera_streamer_create();

// Destroy an Android camera streamer instance
void jr_android_camera_streamer_destroy(jr_android_camera_streamer_t *streamer);

// Initialize the camera with the given parameters
int jr_android_camera_streamer_initialize(
    jr_android_camera_streamer_t *streamer, int32_t width, int32_t height,
    int32_t format);

// Start streaming images
int jr_android_camera_streamer_start_streaming(
    jr_android_camera_streamer_t *streamer);

// Stop streaming images
void jr_android_camera_streamer_stop_streaming(
    jr_android_camera_streamer_t *streamer);

// Callback type for receiving image frames
typedef void (*jr_android_camera_frame_callback_t)(const uint8_t *data,
                                                   size_t size,
                                                   void *user_data);

// Set callback for receiving image frames
void jr_android_camera_streamer_set_frame_callback(
    jr_android_camera_streamer_t *streamer,
    jr_android_camera_frame_callback_t callback, void *user_data);

#ifdef __cplusplus
}
#endif
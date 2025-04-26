#include "camera/android_camera_streamer.h"

#include <android/log.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#define LOG_TAG "AndroidCameraStreamer"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace jr {

class AndroidCameraStreamer::Impl {
public:
  Impl()
      : camera_manager_(nullptr), camera_device_(nullptr),
        capture_session_(nullptr), image_reader_(nullptr),
        frame_callback_(nullptr) {}

  ~Impl() { stopStreaming(); }

  bool initialize(int32_t width, int32_t height, int32_t format) {
    camera_manager_ = ACameraManager_create();
    if (!camera_manager_) {
      LOGE("Failed to create camera manager");
      return false;
    }

    // Get the first available camera
    ACameraIdList *camera_id_list = nullptr;
    if (ACameraManager_getCameraIdList(camera_manager_, &camera_id_list) !=
        ACAMERA_OK) {
      LOGE("Failed to get camera list");
      return false;
    }

    if (camera_id_list->numCameras < 1) {
      LOGE("No cameras available");
      ACameraManager_deleteCameraIdList(camera_id_list);
      return false;
    }

    const char *camera_id = camera_id_list->cameraIds[0];

    // Open the camera
    ACameraDevice_StateCallbacks device_callbacks = {
        .context = this,
        .onDisconnected = onCameraDisconnected,
        .onError = onCameraError,
    };

    if (ACameraManager_openCamera(camera_manager_, camera_id, &device_callbacks,
                                  &camera_device_) != ACAMERA_OK) {
      LOGE("Failed to open camera");
      ACameraManager_deleteCameraIdList(camera_id_list);
      return false;
    }

    ACameraManager_deleteCameraIdList(camera_id_list);

    // Create image reader for capturing frames
    AImageReader *image_reader = nullptr;
    media_status_t status =
        AImageReader_new(width, height, format, MAX_BUF_COUNT, &image_reader);
    if (status != AMEDIA_OK) {
      LOGE("Failed to create image reader");
      return false;
    }

    image_reader_ = image_reader;

    // Set up image reader callback
    AImageReader_ImageListener image_listener{
        .context = this,
        .onImageAvailable = onImageAvailable,
    };
    AImageReader_setImageListener(image_reader_, &image_listener);

    return true;
  }

  bool startStreaming() {
    if (!camera_device_ || !image_reader_) {
      LOGE("Camera or image reader not initialized");
      return false;
    }

    // Get the native window from image reader
    ANativeWindow *window = nullptr;
    media_status_t status = AImageReader_getWindow(image_reader_, &window);
    if (status != AMEDIA_OK) {
      LOGE("Failed to get native window from image reader");
      return false;
    }

    // Create capture session
    ACameraCaptureSession_stateCallbacks session_callbacks = {
        .context = this,
        .onClosed = onSessionClosed,
        .onReady = onSessionReady,
        .onActive = onSessionActive,
    };

    // Create output target
    ACameraOutputTarget *output_target = nullptr;
    if (ACameraOutputTarget_create(window, &output_target) != ACAMERA_OK) {
      LOGE("Failed to create output target");
      return false;
    }

    // Create output container
    ACaptureSessionOutputContainer *output_container = nullptr;
    ACaptureSessionOutputContainer_create(&output_container);
    if (!output_container) {
      LOGE("Failed to create output container");
      ACameraOutputTarget_free(output_target);
      return false;
    }

    // Create capture session
    if (ACameraDevice_createCaptureSession(camera_device_, output_container,
                                           &session_callbacks,
                                           &capture_session_) != ACAMERA_OK) {
      LOGE("Failed to create capture session");
      ACameraOutputTarget_free(output_target);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    // Create capture request
    ACameraCaptureSession_captureCallbacks capture_callbacks = {
        .context = this,
        .onCaptureStarted = onCaptureStarted,
        .onCaptureProgressed = onCaptureProgressed,
        .onCaptureCompleted = onCaptureCompleted,
        .onCaptureFailed = onCaptureFailed,
        .onCaptureSequenceCompleted = onCaptureSequenceCompleted,
        .onCaptureSequenceAborted = onCaptureSequenceAborted,
    };

    // Create and configure capture request
    ACaptureRequest *request = nullptr;
    if (ACameraDevice_createCaptureRequest(camera_device_, TEMPLATE_PREVIEW,
                                           &request) != ACAMERA_OK) {
      LOGE("Failed to create capture request");
      ACameraOutputTarget_free(output_target);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    // Add target to request
    if (ACaptureRequest_addTarget(request, output_target) != ACAMERA_OK) {
      LOGE("Failed to add target to request");
      ACameraOutputTarget_free(output_target);
      ACaptureRequest_free(request);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    // Start repeating request
    if (ACameraCaptureSession_setRepeatingRequest(
            capture_session_, &capture_callbacks, 1, &request, nullptr) !=
        ACAMERA_OK) {
      LOGE("Failed to start repeating request");
      ACameraOutputTarget_free(output_target);
      ACaptureRequest_free(request);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    ACameraOutputTarget_free(output_target);
    ACaptureRequest_free(request);
    ACaptureSessionOutputContainer_free(output_container);

    return true;
  }

  void stopStreaming() {
    if (capture_session_) {
      ACameraCaptureSession_close(capture_session_);
      capture_session_ = nullptr;
    }

    if (image_reader_) {
      AImageReader_delete(image_reader_);
      image_reader_ = nullptr;
    }

    if (camera_device_) {
      ACameraDevice_close(camera_device_);
      camera_device_ = nullptr;
    }

    if (camera_manager_) {
      ACameraManager_delete(camera_manager_);
      camera_manager_ = nullptr;
    }
  }

  void setFrameCallback(std::function<void(const uint8_t *, size_t)> callback) {
    frame_callback_ = callback;
  }

private:
  static const int32_t MAX_BUF_COUNT = 3;

  static void onCameraDisconnected(void *context, ACameraDevice *device) {
    auto impl = static_cast<Impl *>(context);
    LOGE("Camera disconnected");
  }

  static void onCameraError(void *context, ACameraDevice *device, int error) {
    auto impl = static_cast<Impl *>(context);
    LOGE("Camera error: %d", error);
  }

  static void onSessionClosed(void *context, ACameraCaptureSession *session) {
    auto impl = static_cast<Impl *>(context);
    LOGI("Capture session closed");
  }

  static void onSessionReady(void *context, ACameraCaptureSession *session) {
    auto impl = static_cast<Impl *>(context);
    LOGI("Capture session ready");
  }

  static void onSessionActive(void *context, ACameraCaptureSession *session) {
    auto impl = static_cast<Impl *>(context);
    LOGI("Capture session active");
  }

  static void onCaptureStarted(void *context, ACameraCaptureSession *session,
                               const ACaptureRequest *request,
                               int64_t timestamp) {
    auto impl = static_cast<Impl *>(context);
    LOGI("Capture started at %ld", timestamp);
  }

  static void onCaptureProgressed(void *context, ACameraCaptureSession *session,
                                  ACaptureRequest *request,
                                  const ACameraMetadata *result) {
    auto impl = static_cast<Impl *>(context);
  }

  static void onCaptureCompleted(void *context, ACameraCaptureSession *session,
                                 ACaptureRequest *request,
                                 const ACameraMetadata *result) {
    auto impl = static_cast<Impl *>(context);
  }

  static void onCaptureFailed(void *context, ACameraCaptureSession *session,
                              ACaptureRequest *request,
                              ACameraCaptureFailure *failure) {
    auto impl = static_cast<Impl *>(context);
    LOGE("Capture failed with reason: %d", failure->reason);
  }

  static void onCaptureSequenceCompleted(void *context,
                                         ACameraCaptureSession *session,
                                         int sequenceId, int64_t frameNumber) {
    auto impl = static_cast<Impl *>(context);
  }

  static void onCaptureSequenceAborted(void *context,
                                       ACameraCaptureSession *session,
                                       int sequenceId) {
    auto impl = static_cast<Impl *>(context);
    LOGE("Capture sequence aborted");
  }

  static void onImageAvailable(void *context, AImageReader *reader) {
    auto impl = static_cast<Impl *>(context);

    AImage *image = nullptr;
    media_status_t status = AImageReader_acquireLatestImage(reader, &image);
    if (status != AMEDIA_OK) {
      LOGE("Failed to acquire latest image");
      return;
    }

    // Get image format
    int32_t format;
    if (AImage_getFormat(image, &format) != AMEDIA_OK) {
      LOGE("Failed to get image format");
      AImage_delete(image);
      return;
    }

    // Get image dimensions
    int32_t width, height;
    if (AImage_getWidth(image, &width) != AMEDIA_OK ||
        AImage_getHeight(image, &height) != AMEDIA_OK) {
      LOGE("Failed to get image dimensions");
      AImage_delete(image);
      return;
    }

    // Get number of planes
    int32_t num_planes;
    if (AImage_getNumberOfPlanes(image, &num_planes) != AMEDIA_OK) {
      LOGE("Failed to get number of planes");
      AImage_delete(image);
      return;
    }

    // Calculate total data size needed
    size_t total_size = 0;
    for (int i = 0; i < num_planes; i++) {
      int32_t plane_length;
      if (AImage_getPlaneRowStride(image, i, &plane_length) != AMEDIA_OK) {
        LOGE("Failed to get plane %d row stride", i);
        AImage_delete(image);
        return;
      }
      total_size += plane_length * height;
    }

    // Allocate buffer for all planes
    std::vector<uint8_t> combined_data(total_size);
    uint8_t* current_ptr = combined_data.data();

    // Copy data from all planes
    for (int i = 0; i < num_planes; i++) {
      uint8_t *plane_data = nullptr;
      int32_t plane_length = 0;
      if (AImage_getPlaneData(image, i, &plane_data, &plane_length) != AMEDIA_OK) {
        LOGE("Failed to get plane %d data", i);
        AImage_delete(image);
        return;
      }

      // Copy plane data
      memcpy(current_ptr, plane_data, plane_length);
      current_ptr += plane_length;
    }

    // Call the frame callback if set
    if (impl->frame_callback_) {
      impl->frame_callback_(combined_data.data(), total_size);
    }

    AImage_delete(image);
  }

  ACameraManager *camera_manager_;
  ACameraDevice *camera_device_;
  ACameraCaptureSession *capture_session_;
  AImageReader *image_reader_;
  std::function<void(const uint8_t *, size_t)> frame_callback_;
};

AndroidCameraStreamer::AndroidCameraStreamer()
    : impl_(std::make_unique<Impl>()) {}
AndroidCameraStreamer::~AndroidCameraStreamer() = default;

bool AndroidCameraStreamer::initialize(int32_t width, int32_t height,
                                       int32_t format) {
  return impl_->initialize(width, height, format);
}

bool AndroidCameraStreamer::startStreaming() { return impl_->startStreaming(); }

void AndroidCameraStreamer::stopStreaming() { impl_->stopStreaming(); }

void AndroidCameraStreamer::setFrameCallback(
    std::function<void(const uint8_t *, size_t)> callback) {
  impl_->setFrameCallback(callback);
}

} // namespace jr
#include "jr_android/camera_device.hpp"
#include "jr_imgproc/image.h"

#include <android/log.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#define LOG_TAG "JRAndroidCameraDevice"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace jr::android {

class CameraDevice::Impl {
public:
  Impl()
      : camera_manager_(nullptr)
      , camera_device_(nullptr)
      , capture_session_(nullptr)
      , image_reader_(nullptr)
      , frame_callback_(nullptr) {
    camera_manager_ = ACameraManager_create();
    if (!camera_manager_) {
      LOGE("Failed to create camera manager");
    }
  }

  ~Impl() {
    stopStreaming();

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

  int getNumberOfCameras() const {
    ACameraIdList* camera_id_list = nullptr;
    if (ACameraManager_getCameraIdList(camera_manager_, &camera_id_list) !=
        ACAMERA_OK) {
      LOGE("Failed to get camera list");
      return 0;
    }

    int num_cameras = camera_id_list->numCameras;
    ACameraManager_deleteCameraIdList(camera_id_list);
    return num_cameras;
  }

  bool open(int width, int height, int camera_idx) {
    ACameraIdList* camera_id_list = nullptr;
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

    const char* camera_id = camera_id_list->cameraIds[camera_idx];

    // Open the camera
    ACameraDevice_StateCallbacks device_callbacks = {
      .context = this,
      .onDisconnected = onCameraDisconnected,
      .onError = onCameraError,
    };

    if (ACameraManager_openCamera(
          camera_manager_,
          camera_id,
          &device_callbacks,
          &camera_device_
        ) != ACAMERA_OK) {
      LOGE("Failed to open camera");
      ACameraManager_deleteCameraIdList(camera_id_list);
      return false;
    }

    ACameraManager_deleteCameraIdList(camera_id_list);

    // Create image reader for capturing frames
    media_status_t status = AImageReader_new(
      width,
      height,
      IMAGE_FORMAT,
      MAX_BUF_COUNT,
      &image_reader_
    );
    if (status != AMEDIA_OK) {
      LOGE("Failed to create image reader");
      ACameraDevice_close(camera_device_);
      camera_device_ = nullptr;
      return false;
    }

    // Set up image reader callback
    AImageReader_ImageListener image_listener{
      .context = this,
      .onImageAvailable = onImageAvailable,
    };
    AImageReader_setImageListener(image_reader_, &image_listener);

    return true;
  }

  void close() {
    if (image_reader_) {
      AImageReader_delete(image_reader_);
      image_reader_ = nullptr;
    }

    if (camera_device_) {
      ACameraDevice_close(camera_device_);
      camera_device_ = nullptr;
    }
  }

  bool startStreaming() {
    if (!camera_device_ || !image_reader_) {
      LOGE("Camera or image reader not initialized");
      return false;
    }

    // Get the native window from image reader
    ANativeWindow* window = nullptr;
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
    ACameraOutputTarget* output_target = nullptr;
    if (ACameraOutputTarget_create(window, &output_target) != ACAMERA_OK) {
      LOGE("Failed to create output target");
      return false;
    }

    // Create output container
    ACaptureSessionOutputContainer* output_container = nullptr;
    ACaptureSessionOutputContainer_create(&output_container);
    if (!output_container) {
      LOGE("Failed to create output container");
      ACameraOutputTarget_free(output_target);
      return false;
    }

    // Add output target to container
    ACaptureSessionOutput* session_output = nullptr;
    if (ACaptureSessionOutput_create(window, &session_output) != ACAMERA_OK) {
      LOGE("Failed to create session output");
      ACameraOutputTarget_free(output_target);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    if (ACaptureSessionOutputContainer_add(output_container, session_output) !=
        ACAMERA_OK) {
      LOGE("Failed to add session output to container");
      ACameraOutputTarget_free(output_target);
      ACaptureSessionOutputContainer_free(output_container);
      ACaptureSessionOutput_free(session_output);
      return false;
    }

    // Create capture session
    if (ACameraDevice_createCaptureSession(
          camera_device_,
          output_container,
          &session_callbacks,
          &capture_session_
        ) != ACAMERA_OK) {
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
    ACaptureRequest* request = nullptr;
    if (ACameraDevice_createCaptureRequest(
          camera_device_,
          TEMPLATE_PREVIEW,
          &request
        ) != ACAMERA_OK) {
      LOGE("Failed to create capture request");
      ACameraOutputTarget_free(output_target);
      ACaptureSessionOutput_free(session_output);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    // Add target to request
    if (ACaptureRequest_addTarget(request, output_target) != ACAMERA_OK) {
      LOGE("Failed to add target to request");
      ACameraOutputTarget_free(output_target);
      ACaptureRequest_free(request);
      ACaptureSessionOutput_free(session_output);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    // Start repeating request
    if (ACameraCaptureSession_setRepeatingRequest(
          capture_session_,
          &capture_callbacks,
          1,
          &request,
          nullptr
        ) != ACAMERA_OK) {
      LOGE("Failed to start repeating request");
      ACameraOutputTarget_free(output_target);
      ACaptureRequest_free(request);
      ACaptureSessionOutput_free(session_output);
      ACaptureSessionOutputContainer_free(output_container);
      return false;
    }

    ACameraOutputTarget_free(output_target);
    ACaptureRequest_free(request);
    ACaptureSessionOutput_free(session_output);
    ACaptureSessionOutputContainer_free(output_container);

    return true;
  }

  void stopStreaming() {
    if (capture_session_) {
      // Try to stop repeating requests first
      if (ACameraCaptureSession_stopRepeating(capture_session_) != ACAMERA_OK) {
        LOGI("Failed to stop repeating requests, continuing with cleanup");
      }
    }
  }

  void setFrameCallback(FrameCallback callback) {
    frame_callback_ = callback;
  }

  void setSessionReadyCallback(VoidCallback callback) {
    session_ready_callback_ = callback;
  }

private:
  static const int32_t MAX_BUF_COUNT = 3;
  static const AIMAGE_FORMATS IMAGE_FORMAT = AIMAGE_FORMAT_YUV_420_888;

  static void onCameraDisconnected(void* context, ACameraDevice* device) {
    auto impl = static_cast<Impl*>(context);
    LOGE("Camera disconnected unexpectedly");
    impl->stopStreaming();
  }

  static void onCameraError(void* context, ACameraDevice* device, int error) {
    auto impl = static_cast<Impl*>(context);
    LOGE("Camera error: %d", error);
    impl->stopStreaming();
  }

  static void onSessionClosed(void* context, ACameraCaptureSession* session) {
    auto impl = static_cast<Impl*>(context);
    LOGI("Capture session closed");
    impl->capture_session_ = nullptr;
  }

  static void onSessionReady(void* context, ACameraCaptureSession* session) {
    auto impl = static_cast<Impl*>(context);
    LOGI("Capture session ready");
    if (impl->session_ready_callback_) {
      impl->session_ready_callback_();
    }
  }

  static void onSessionActive(void* context, ACameraCaptureSession* session) {
    auto impl = static_cast<Impl*>(context);
    LOGI("Capture session active");
  }

  static void onCaptureStarted(
    void* context,
    ACameraCaptureSession* session,
    const ACaptureRequest* request,
    int64_t timestamp
  ) {
    auto impl = static_cast<Impl*>(context);
    // LOGI("Capture started at %ld", timestamp);
  }

  static void onCaptureProgressed(
    void* context,
    ACameraCaptureSession* session,
    ACaptureRequest* request,
    const ACameraMetadata* result
  ) {
    auto impl = static_cast<Impl*>(context);
  }

  static void onCaptureCompleted(
    void* context,
    ACameraCaptureSession* session,
    ACaptureRequest* request,
    const ACameraMetadata* result
  ) {
    auto impl = static_cast<Impl*>(context);
  }

  static void onCaptureFailed(
    void* context,
    ACameraCaptureSession* session,
    ACaptureRequest* request,
    ACameraCaptureFailure* failure
  ) {
    auto impl = static_cast<Impl*>(context);
    LOGE(
      "Capture failed with reason: %d, frame number: %ld",
      failure->reason,
      failure->frameNumber
    );
    if (failure->reason == ACAMERA_ERROR_CAMERA_DEVICE) {
      impl->stopStreaming();
    }
  }

  static void onCaptureSequenceCompleted(
    void* context,
    ACameraCaptureSession* session,
    int sequenceId,
    int64_t frameNumber
  ) {
    auto impl = static_cast<Impl*>(context);
  }

  static void onCaptureSequenceAborted(
    void* context,
    ACameraCaptureSession* session,
    int sequenceId
  ) {
    auto impl = static_cast<Impl*>(context);
    LOGE("Capture sequence aborted");
  }

  static void onImageAvailable(void* context, AImageReader* reader) {
    if (!context || !reader) {
      LOGE("Invalid context or reader in onImageAvailable");
      return;
    }

    auto* impl = static_cast<Impl*>(context);
    if (!impl) {
      LOGE("Failed to cast context to Impl");
      return;
    }

    AImage* image = nullptr;
    if (AImageReader_acquireLatestImage(reader, &image) != AMEDIA_OK) {
      LOGE("Failed to acquire latest image");
      return;
    }

    int32_t format;
    int32_t width, height;
    int32_t num_planes;
    jr_planar_image_t planar_image{};
    bool success = true;

    // Get image format
    if (AImage_getFormat(image, &format) != AMEDIA_OK) {
      LOGE("Failed to get image format");
      success = false;
      goto IMAGE_READ_ERROR;
    }

    // Verify format is YUV420
    if (format != AIMAGE_FORMAT_YUV_420_888) {
      LOGE("Unexpected image format: %d, expected YUV_420_888", format);
      success = false;
      goto IMAGE_READ_ERROR;
    }

    // Get image dimensions
    if (AImage_getWidth(image, &width) != AMEDIA_OK ||
        AImage_getHeight(image, &height) != AMEDIA_OK) {
      LOGE("Failed to get image dimensions");
      success = false;
      goto IMAGE_READ_ERROR;
    }

    // Get number of planes
    if (AImage_getNumberOfPlanes(image, &num_planes) != AMEDIA_OK) {
      LOGE("Failed to get number of planes");
      success = false;
      goto IMAGE_READ_ERROR;
    }

    // YUV_420_888 should have 3 planes (Y, U, V)
    if (num_planes != 3) {
      LOGE("Unexpected number of planes: %d, expected 3", num_planes);
      success = false;
      goto IMAGE_READ_ERROR;
    }

    planar_image = {
      .width = width,
      .height = height,
      .num_planes = num_planes,
      .planes = new jr_plane_t[num_planes]
    };

    for (int i = 0; i < num_planes; i++) {
      int32_t row_stride;
      if (AImage_getPlaneRowStride(image, i, &row_stride) != AMEDIA_OK) {
        LOGE("Failed to get plane row stride");
        success = false;
        goto IMAGE_READ_ERROR;
      }

      int32_t pixel_stride;
      if (AImage_getPlanePixelStride(image, i, &pixel_stride) != AMEDIA_OK) {
        LOGE("Failed to get plane pixel stride");
        success = false;
        goto IMAGE_READ_ERROR;
      }

      uint8_t* data;
      int32_t length;
      if (AImage_getPlaneData(image, i, &data, &length) != AMEDIA_OK) {
        LOGE("Failed to get plane data");
        success = false;
        goto IMAGE_READ_ERROR;
      }

      planar_image.planes[i].data = data;
      planar_image.planes[i].row_stride = row_stride;
      planar_image.planes[i].pixel_stride = pixel_stride;
    }

    // Calculate the size of the RGBA data
    if (success && impl->frame_callback_) {
      impl->frame_callback_(planar_image);
    }

  IMAGE_READ_ERROR:
    // Free the allocated planar image
    if (planar_image.planes) {
      delete[] planar_image.planes;
    }

    // Always delete the image before returning
    AImage_delete(image);
  }

  ACameraManager* camera_manager_;
  ACameraDevice* camera_device_;
  ACameraCaptureSession* capture_session_;
  AImageReader* image_reader_;
  FrameCallback frame_callback_;
  VoidCallback session_ready_callback_;
};

CameraDevice::CameraDevice()
    : impl_(std::make_unique<Impl>()) {
}

CameraDevice::~CameraDevice() = default;

int CameraDevice::getNumberOfCameras() const {
  return impl_->getNumberOfCameras();
}

bool CameraDevice::open(int width, int height, int camera_idx) {
  return impl_->open(width, height, camera_idx);
}

void CameraDevice::close() {
  impl_->close();
}

bool CameraDevice::startStreaming() {
  return impl_->startStreaming();
}

void CameraDevice::stopStreaming() {
  impl_->stopStreaming();
}

void CameraDevice::setFrameCallback(FrameCallback callback) {
  impl_->setFrameCallback(callback);
}

void CameraDevice::setSessionReadyCallback(VoidCallback callback) {
  impl_->setSessionReadyCallback(callback);
}

} // namespace jr::android
#include "jr_android/imu_device.hpp"

#include <android/log.h>
#include <android/looper.h>
#include <android/sensor.h>

#include <atomic>
#include <mutex>
#include <thread>

#define LOG_TAG "JRAndroidImuDevice"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace jr::android {

class ImuDevice::Impl {
public:
  Impl()
      : sensor_manager_(nullptr)
      , accelerometer_(nullptr)
      , gyroscope_(nullptr)
      , event_queue_(nullptr)
      , looper_(nullptr)
      , running_(false) {
    sensor_manager_ = ASensorManager_getInstance();
    if (!sensor_manager_) {
      LOGE("Failed to get sensor manager");
      return;
    }

    accelerometer_ = ASensorManager_getDefaultSensor(
      sensor_manager_,
      ASENSOR_TYPE_ACCELEROMETER
    );
    if (!accelerometer_) {
      LOGI("Accelerometer not available");
    }

    gyroscope_ =
      ASensorManager_getDefaultSensor(sensor_manager_, ASENSOR_TYPE_GYROSCOPE);
    if (!gyroscope_) {
      LOGI("Gyroscope not available");
    }
  }

  ~Impl() {
    stop();
  }

  bool isAvailable() const {
    return sensor_manager_ && (accelerometer_ || gyroscope_);
  }

  bool start() {
    if (!isAvailable()) {
      LOGE("IMU sensors not available");
      return false;
    }

    if (running_.load()) {
      LOGI("IMU already running");
      return true;
    }

    running_.store(true);
    sensor_thread_ = std::thread(&Impl::sensorLoop, this);
    return true;
  }

  void stop() {
    running_.store(false);
    if (sensor_thread_.joinable()) {
      sensor_thread_.join();
    }
  }

  void setImuCallback(ImuCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
  }

private:
  static constexpr int LOOPER_ID_SENSOR = 1;
  static constexpr int SENSOR_RATE_US = 10000; // 100 Hz (10ms)

  void sensorLoop() {
    // Create looper for this thread
    looper_ = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    if (!looper_) {
      LOGE("Failed to prepare looper");
      return;
    }

    // Create event queue
    event_queue_ = ASensorManager_createEventQueue(
      sensor_manager_,
      looper_,
      LOOPER_ID_SENSOR,
      nullptr,
      nullptr
    );
    if (!event_queue_) {
      LOGE("Failed to create sensor event queue");
      return;
    }

    // Enable sensors
    if (accelerometer_) {
      if (ASensorEventQueue_enableSensor(event_queue_, accelerometer_) < 0) {
        LOGE("Failed to enable accelerometer");
      } else {
        ASensorEventQueue_setEventRate(
          event_queue_,
          accelerometer_,
          SENSOR_RATE_US
        );
        LOGI("Accelerometer enabled");
      }
    }

    if (gyroscope_) {
      if (ASensorEventQueue_enableSensor(event_queue_, gyroscope_) < 0) {
        LOGE("Failed to enable gyroscope");
      } else {
        ASensorEventQueue_setEventRate(
          event_queue_,
          gyroscope_,
          SENSOR_RATE_US
        );
        LOGI("Gyroscope enabled");
      }
    }

    ImuData current_data{};

    while (running_.load()) {
      // Poll for sensor events with timeout
      int ident = ALooper_pollOnce(
        100, // 100ms timeout
        nullptr,
        nullptr,
        nullptr
      );

      if (ident == LOOPER_ID_SENSOR) {
        ASensorEvent event;
        while (ASensorEventQueue_getEvents(event_queue_, &event, 1) > 0) {
          if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
            current_data.ax = event.acceleration.x;
            current_data.ay = event.acceleration.y;
            current_data.az = event.acceleration.z;
            current_data.timestamp_ns = event.timestamp;
          } else if (event.type == ASENSOR_TYPE_GYROSCOPE) {
            current_data.gx = event.gyro.x;
            current_data.gy = event.gyro.y;
            current_data.gz = event.gyro.z;
            current_data.timestamp_ns = event.timestamp;
          }

          // Invoke callback
          {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (callback_) {
              callback_(current_data);
            }
          }
        }
      }
    }

    // Disable sensors and cleanup
    if (accelerometer_) {
      ASensorEventQueue_disableSensor(event_queue_, accelerometer_);
    }
    if (gyroscope_) {
      ASensorEventQueue_disableSensor(event_queue_, gyroscope_);
    }

    if (event_queue_) {
      ASensorManager_destroyEventQueue(sensor_manager_, event_queue_);
      event_queue_ = nullptr;
    }
  }

  ASensorManager* sensor_manager_;
  const ASensor* accelerometer_;
  const ASensor* gyroscope_;
  ASensorEventQueue* event_queue_;
  ALooper* looper_;

  std::atomic<bool> running_;
  std::thread sensor_thread_;

  std::mutex callback_mutex_;
  ImuCallback callback_;
};

ImuDevice::ImuDevice()
    : impl_(std::make_unique<Impl>()) {
}

ImuDevice::~ImuDevice() = default;

bool ImuDevice::start() {
  return impl_->start();
}

void ImuDevice::stop() {
  impl_->stop();
}

bool ImuDevice::isAvailable() const {
  return impl_->isAvailable();
}

void ImuDevice::setImuCallback(ImuCallback callback) {
  impl_->setImuCallback(std::move(callback));
}

} // namespace jr::android

#pragma once

#include <functional>
#include <memory>

namespace jr::android {

struct ImuData {
  // Linear acceleration (m/s^2)
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;

  // Angular velocity (rad/s)
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;

  // Timestamp in nanoseconds
  uint64_t timestamp_ns = 0;
};

using ImuCallback = std::function<void(const ImuData&)>;

class ImuDevice {
public:
  ImuDevice();
  ~ImuDevice();

  // Start acquiring IMU data
  bool start();

  // Stop acquiring IMU data
  void stop();

  // Check if IMU is available
  bool isAvailable() const;

  // Set callback for receiving IMU data
  void setImuCallback(ImuCallback callback);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace jr::android

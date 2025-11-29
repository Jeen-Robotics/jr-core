#pragma once

#include <sensor_msgs.pb.h>

#include <opencv2/core.hpp>

namespace jr::vio {

/**
 * IMU pre-integration state.
 * Maintains velocity, position, and rotation estimates from IMU measurements.
 */
struct ImuState {
  cv::Vec3d velocity{0, 0, 0};
  cv::Vec3d position{0, 0, 0};
  cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);

  // Bias estimates
  cv::Vec3d gyro_bias{0, 0, 0};
  cv::Vec3d accel_bias{0, 0, 0};

  double last_timestamp = -1.0;

  /**
   * Integrate a single IMU measurement.
   */
  void integrate(const sensor_msgs::Imu& imu);

  /**
   * Reset state to initial values.
   */
  void reset();
};

} // namespace jr::vio

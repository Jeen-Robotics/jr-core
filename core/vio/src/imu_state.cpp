#include "vio/imu_state.hpp"
#include "vio/msg_utils.hpp"

#include <opencv2/calib3d.hpp>

namespace jr::vio {

void ImuState::integrate(const sensor_msgs::Imu& imu) {
  double t = get_timestamp_sec(imu);
  if (last_timestamp < 0) {
    last_timestamp = t;
    return;
  }

  double dt = t - last_timestamp;
  if (dt <= 0 || dt > 0.1) {
    last_timestamp = t;
    return;
  }

  // Get measurements with bias correction
  cv::Vec3d gyro(
    imu.angular_velocity().x() - gyro_bias[0],
    imu.angular_velocity().y() - gyro_bias[1],
    imu.angular_velocity().z() - gyro_bias[2]
  );

  cv::Vec3d accel(
    imu.linear_acceleration().x() - accel_bias[0],
    imu.linear_acceleration().y() - accel_bias[1],
    imu.linear_acceleration().z() - accel_bias[2]
  );

  // Gravity compensation (assuming device is roughly level)
  cv::Vec3d gravity(0, 0, 9.81);

  // Rotate acceleration to world frame and remove gravity
  cv::Mat accel_mat = (cv::Mat_<double>(3, 1) << accel[0], accel[1], accel[2]);
  cv::Mat accel_world = rotation * accel_mat;
  cv::Vec3d accel_w(
    accel_world.at<double>(0) - gravity[0],
    accel_world.at<double>(1) - gravity[1],
    accel_world.at<double>(2) - gravity[2]
  );

  // Simple Euler integration
  velocity += accel_w * dt;
  position += velocity * dt + 0.5 * accel_w * dt * dt;

  // Rotation update using Rodrigues formula
  double angle = cv::norm(gyro) * dt;
  if (angle > 1e-10) {
    cv::Vec3d axis = gyro / cv::norm(gyro);
    cv::Mat axis_mat = (cv::Mat_<double>(3, 1) << axis[0], axis[1], axis[2]);
    cv::Mat dR;
    cv::Rodrigues(axis_mat * angle, dR);
    rotation = rotation * dR;
  }

  last_timestamp = t;
}

void ImuState::reset() {
  velocity = cv::Vec3d(0, 0, 0);
  position = cv::Vec3d(0, 0, 0);
  rotation = cv::Mat::eye(3, 3, CV_64F);
  last_timestamp = -1.0;
}

} // namespace jr::vio

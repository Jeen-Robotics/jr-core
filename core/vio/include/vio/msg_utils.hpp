#pragma once

#include <geometry_msgs.pb.h>
#include <sensor_msgs.pb.h>

#include <opencv2/core.hpp>

namespace jr::vio {

/**
 * Deserialize image encoding string to OpenCV type.
 */
int deserialize_encoding(const std::string& encoding);

/**
 * Serialize OpenCV type to image encoding string.
 */
std::string serialize_encoding(int cv_type);

/**
 * Convert sensor_msgs::Image to cv::Mat.
 * Returns a cloned copy of the image data.
 */
cv::Mat from_image_msg(const sensor_msgs::Image& msg);

/**
 * Get timestamp in seconds from sensor_msgs::Image header.
 */
double get_timestamp_sec(const sensor_msgs::Image& msg);

/**
 * Get timestamp in seconds from sensor_msgs::Imu header.
 */
double get_timestamp_sec(const sensor_msgs::Imu& msg);

/**
 * Create a PoseStamped message from rotation matrix and translation vector.
 */
geometry_msgs::PoseStamped create_pose_msg(
  const cv::Mat& R,
  const cv::Mat& t,
  double timestamp_sec
);

/**
 * Convert quaternion to rotation matrix.
 */
cv::Mat quaternion_to_rotation_matrix(
  double qx,
  double qy,
  double qz,
  double qw
);

/**
 * Convert rotation matrix to quaternion.
 * Returns (qx, qy, qz, qw).
 */
std::tuple<double, double, double, double> rotation_matrix_to_quaternion(
  const cv::Mat& R
);

} // namespace jr::vio

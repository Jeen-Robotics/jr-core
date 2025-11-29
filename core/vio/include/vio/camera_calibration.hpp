#pragma once

#include <geometry_msgs.pb.h>
#include <sensor_msgs.pb.h>

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace jr::vio {

/**
 * Camera calibration parameters including intrinsics and extrinsics.
 */
struct CameraCalibration {
  // Intrinsics
  double fx = 500.0;
  double fy = 500.0;
  double cx = 320.0;
  double cy = 240.0;
  int width = 640;
  int height = 480;

  // Distortion coefficients (plumb_bob: k1, k2, p1, p2, k3)
  std::vector<double> distortion_coeffs;
  std::string distortion_model = "plumb_bob";

  // Extrinsics: Camera pose relative to IMU (T_imu_camera)
  cv::Mat R_imu_camera = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t_imu_camera = cv::Mat::zeros(3, 1, CV_64F);

  // Flags
  bool has_intrinsics = false;
  bool has_extrinsics = false;

  /**
   * Get the 3x3 camera intrinsic matrix K.
   */
  cv::Mat get_camera_matrix() const;

  /**
   * Get distortion coefficients as cv::Mat.
   */
  cv::Mat get_distortion_coeffs() const;

  /**
   * Set intrinsics from CameraInfo message.
   */
  void set_intrinsics(const sensor_msgs::CameraInfo& msg);

  /**
   * Set extrinsics from PoseStamped message.
   */
  void set_extrinsics(const geometry_msgs::PoseStamped& msg);
};

} // namespace jr::vio

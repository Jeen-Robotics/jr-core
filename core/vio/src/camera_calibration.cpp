#include "vio/camera_calibration.hpp"
#include "vio/msg_utils.hpp"

#include <iostream>

namespace jr::vio {

cv::Mat CameraCalibration::get_camera_matrix() const {
  return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}

cv::Mat CameraCalibration::get_distortion_coeffs() const {
  if (distortion_coeffs.empty()) {
    return cv::Mat::zeros(5, 1, CV_64F);
  }
  cv::Mat coeffs(static_cast<int>(distortion_coeffs.size()), 1, CV_64F);
  for (size_t i = 0; i < distortion_coeffs.size(); ++i) {
    coeffs.at<double>(static_cast<int>(i)) = distortion_coeffs[i];
  }
  return coeffs;
}

void CameraCalibration::set_intrinsics(const sensor_msgs::CameraInfo& msg) {
  if (msg.k_size() >= 9) {
    fx = msg.k(0);
    fy = msg.k(4);
    cx = msg.k(2);
    cy = msg.k(5);
    width = static_cast<int>(msg.width());
    height = static_cast<int>(msg.height());
    has_intrinsics = true;

    std::cout << "Camera intrinsics received:" << std::endl;
    std::cout << "  fx=" << fx << ", fy=" << fy << std::endl;
    std::cout << "  cx=" << cx << ", cy=" << cy << std::endl;
    std::cout << "  size=" << width << "x" << height << std::endl;
  }

  // Get distortion coefficients
  if (msg.d_size() > 0) {
    distortion_coeffs.clear();
    for (int i = 0; i < msg.d_size(); ++i) {
      distortion_coeffs.push_back(msg.d(i));
    }
    distortion_model = msg.distortion_model();
    std::cout << "  distortion (" << distortion_model << "): [";
    for (size_t i = 0; i < distortion_coeffs.size(); ++i) {
      std::cout << distortion_coeffs[i];
      if (i < distortion_coeffs.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }
}

void CameraCalibration::set_extrinsics(const geometry_msgs::PoseStamped& msg) {
  // Extract position
  t_imu_camera.at<double>(0) = msg.pose().position().x();
  t_imu_camera.at<double>(1) = msg.pose().position().y();
  t_imu_camera.at<double>(2) = msg.pose().position().z();

  // Extract orientation
  double qx = msg.pose().orientation().x();
  double qy = msg.pose().orientation().y();
  double qz = msg.pose().orientation().z();
  double qw = msg.pose().orientation().w();

  R_imu_camera = quaternion_to_rotation_matrix(qx, qy, qz, qw);
  has_extrinsics = true;

  std::cout << "Camera extrinsics received (T_imu_camera):" << std::endl;
  std::cout << "  position: (" << t_imu_camera.at<double>(0) << ", "
            << t_imu_camera.at<double>(1) << ", " << t_imu_camera.at<double>(2)
            << ")" << std::endl;
  std::cout << "  orientation (quat): (" << qx << ", " << qy << ", " << qz
            << ", " << qw << ")" << std::endl;
}

} // namespace jr::vio

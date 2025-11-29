#pragma once

#include "vio/camera_calibration.hpp"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

namespace jr::vio {

/**
 * Visual odometry state.
 * Maintains the current pose, feature tracking state, and trajectory history.
 */
struct VioState {
  // Camera calibration
  CameraCalibration calibration;

  // Current pose in world frame
  cv::Mat R_world = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t_world = cv::Mat::zeros(3, 1, CV_64F);

  // Previous frame data
  cv::Mat prev_gray;
  std::vector<cv::Point2f> prev_points;
  std::vector<cv::KeyPoint> prev_keypoints;

  // Trajectory history
  std::vector<cv::Point3d> trajectory;

  // Feature detector
  cv::Ptr<cv::ORB> orb;
  int max_features = 500;
  int min_features = 100;

  // Tracking statistics
  int tracked_count = 0;
  int detected_count = 0;

  VioState();

  /**
   * Get the camera intrinsic matrix.
   */
  cv::Mat get_camera_matrix() const;
};

} // namespace jr::vio

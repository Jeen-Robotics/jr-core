#pragma once

#include <opencv2/core.hpp>

#include <mutex>
#include <vector>

namespace jr::vio {

/**
 * Visualization state shared between VIO node and visualizer.
 * Protected by mutex for thread-safe access.
 */
struct VisState {
  std::mutex mutex;

  cv::Mat frame;          // Current frame with features drawn
  cv::Mat trajectory_img; // Top-down trajectory view

  std::vector<cv::Point2f> current_points;
  std::vector<cv::Point2f> prev_points;
  std::vector<uchar> tracking_status;
  std::vector<cv::Point3d> trajectory_3d;

  cv::Point3d current_position{0, 0, 0};
  int tracked_features = 0;
  int detected_features = 0;
  double fps = 0.0;

  // Calibration status
  bool has_intrinsics = false;
  bool has_extrinsics = false;
  double fx = 0, fy = 0, cx = 0, cy = 0;

  bool has_new_data = false;
};

} // namespace jr::vio

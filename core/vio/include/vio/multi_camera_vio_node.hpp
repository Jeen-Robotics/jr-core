#pragma once

#include "vio/imu_state.hpp"
#include "vio/vio_state.hpp"

#include <geometry_msgs.pb.h>
#include <middleware/node.hpp>
#include <middleware/publisher.hpp>
#include <middleware/subscription.hpp>
#include <sensor_msgs.pb.h>

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

namespace jr::vio {

// Maximum number of cameras supported
constexpr int MAX_CAMERAS = 3;

// Camera configuration
// Camera 0: Back wide-angle
// Camera 1: Front-facing (excluded from VIO fusion)
// Camera 2: Back telephoto
constexpr int FRONT_CAMERA = 1;
constexpr int BACK_CAM_WIDE = 0;
constexpr int BACK_CAM_TELE = 2;

/**
 * 3D point with color for pointcloud visualization.
 */
struct ColorPoint3D {
  cv::Point3d position;
  cv::Vec3b color;
  double confidence = 1.0;
};

/**
 * Per-camera tracking state for multi-camera VIO.
 */
struct CameraTrackingState {
  VioState vio;
  cv::Mat frame;                      // Latest frame (color)
  cv::Mat gray;                       // Latest grayscale frame
  std::vector<cv::Point2f> good_prev; // Good previous points
  std::vector<cv::Point2f> good_curr; // Good current points
  std::vector<float> flow_magnitude;  // Optical flow magnitude per point
  std::chrono::steady_clock::time_point last_frame_time;
  uint64_t frame_timestamp_ns = 0;
  bool first_frame = true;
  bool active = false;
  double fps = 0.0;
};

/**
 * Multi-camera visualization state.
 */
struct MultiCameraVisState {
  std::mutex mutex;

  // Per-camera frames with features drawn
  std::array<cv::Mat, MAX_CAMERAS> frames;
  std::array<bool, MAX_CAMERAS> frame_valid{false, false, false};

  // Per-camera feature counts
  std::array<int, MAX_CAMERAS> tracked_features{0, 0, 0};
  std::array<int, MAX_CAMERAS> detected_features{0, 0, 0};
  std::array<double, MAX_CAMERAS> fps{0, 0, 0};

  // Per-camera calibration status
  std::array<bool, MAX_CAMERAS> has_intrinsics{false, false, false};
  std::array<bool, MAX_CAMERAS> has_extrinsics{false, false, false};

  // Combined trajectory
  std::vector<cv::Point3d> trajectory_3d;
  cv::Point3d current_position{0, 0, 0};

  // Active camera count
  int active_cameras = 0;

  // Depth map from optical flow (inverse depth proxy)
  cv::Mat sparse_depth_map;  // Sparse depth from tracked features
  cv::Mat dense_depth_map;   // Dense depth from Farneback optical flow
  cv::Mat flow_hsv;          // Flow visualization (HSV: hue=direction, value=magnitude)
  bool has_depth = false;
  double min_flow = 0.0;
  double max_flow = 10.0;

  // Accumulated 3D pointcloud (from monocular depth estimation)
  std::vector<ColorPoint3D> pointcloud;
  static constexpr size_t MAX_POINTCLOUD_SIZE = 50000;

  // Pointcloud visualization images
  cv::Mat pointcloud_top_view;
  cv::Mat pointcloud_side_view;

  bool has_new_data = false;
};

/**
 * Multi-Camera Visual-Inertial Odometry Node.
 *
 * Uses back cameras (0 and 2) for VIO fusion.
 * Front camera (1) is displayed but excluded from pose estimation.
 * Depth is estimated from optical flow magnitude (motion parallax).
 */
class MultiCameraVioNode final : public mw::Node {
public:
  explicit MultiCameraVioNode(std::shared_ptr<MultiCameraVisState> vis_state);

private:
  std::shared_ptr<MultiCameraVisState> vis_state_;

  // Per-camera subscriptions
  std::array<mw::Subscription, MAX_CAMERAS> sub_images_;
  std::array<mw::Subscription, MAX_CAMERAS> sub_camera_infos_;
  std::array<mw::Subscription, MAX_CAMERAS> sub_camera_poses_;
  mw::Subscription sub_imu_;

  mw::Publisher<geometry_msgs::PoseStamped> pub_odom_;

  // Per-camera tracking state
  std::array<CameraTrackingState, MAX_CAMERAS> cameras_;

  // Shared IMU state
  ImuState imu_;

  // Fused world pose (in IMU frame)
  cv::Mat R_world_ = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t_world_ = cv::Mat::zeros(3, 1, CV_64F);
  std::vector<cv::Point3d> trajectory_;

  // Mutex for pose updates
  std::mutex pose_mutex_;

  // Frame counter for depth estimation frequency
  int frame_counter_ = 0;

  void on_camera_info(int cam_idx, const sensor_msgs::CameraInfo& msg);
  void on_camera_pose(int cam_idx, const geometry_msgs::PoseStamped& msg);
  void on_imu(const sensor_msgs::Imu& msg);
  void on_image(int cam_idx, const sensor_msgs::Image& msg);

  void detect_features(int cam_idx, const cv::Mat& gray);
  void update_visualization(int cam_idx, const cv::Mat& frame);

  // Fuse motion estimates from back cameras only
  void fuse_motion_estimates(
    const std::vector<std::pair<int, std::pair<cv::Mat, cv::Mat>>>& estimates,
    double timestamp
  );

  // Transform camera-frame motion to IMU frame
  std::pair<cv::Mat, cv::Mat> transform_to_imu_frame(
    int cam_idx,
    const cv::Mat& R_cam,
    const cv::Mat& t_cam
  );

  // Compute sparse depth from tracked features
  void compute_sparse_depth(int cam_idx);

  // Compute dense depth using Farneback optical flow
  void compute_dense_depth(int cam_idx);

  // Triangulate 3D points from optical flow and motion
  void triangulate_from_motion(int cam_idx);

  // Add points to accumulated pointcloud
  void add_to_pointcloud(const std::vector<ColorPoint3D>& new_points);
};

} // namespace jr::vio

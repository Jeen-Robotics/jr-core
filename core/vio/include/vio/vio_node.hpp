#pragma once

#include "vio/imu_state.hpp"
#include "vio/vio_state.hpp"
#include "vio/vis_state.hpp"

#include <geometry_msgs.pb.h>
#include <middleware/node.hpp>
#include <middleware/publisher.hpp>
#include <middleware/subscription.hpp>
#include <sensor_msgs.pb.h>

#include <chrono>
#include <memory>

namespace jr::vio {

/**
 * Visual-Inertial Odometry Node.
 *
 * Subscribes to:
 * - /cameraN/frame (sensor_msgs::Image)
 * - /cameraN/frame/info (sensor_msgs::CameraInfo)
 * - /cameraN/frame/pose (geometry_msgs::PoseStamped)
 * - /imu (sensor_msgs::Imu)
 *
 * Publishes:
 * - /vio/odometry (geometry_msgs::PoseStamped)
 */
class VioNode final : public mw::Node {
public:
  explicit VioNode(std::shared_ptr<VisState> vis_state, int camera_idx = 0);

private:
  std::shared_ptr<VisState> vis_state_;
  int camera_idx_;

  mw::Subscription sub_image_;
  mw::Subscription sub_imu_;
  mw::Subscription sub_camera_info_;
  mw::Subscription sub_camera_pose_;
  mw::Publisher<geometry_msgs::PoseStamped> pub_odom_;

  VioState vio_;
  ImuState imu_;

  std::chrono::steady_clock::time_point last_frame_time_;
  bool first_frame_ = true;

  void on_camera_info(const sensor_msgs::CameraInfo& msg);
  void on_camera_pose(const geometry_msgs::PoseStamped& msg);
  void on_imu(const sensor_msgs::Imu& msg);
  void on_image(const sensor_msgs::Image& msg);

  void detect_features(const cv::Mat& gray);
  void update_visualization(const cv::Mat& frame);
};

} // namespace jr::vio

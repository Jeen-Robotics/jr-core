#include "vio/vio_node.hpp"
#include "vio/msg_utils.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <iostream>

namespace jr::vio {

VioNode::VioNode(std::shared_ptr<VisState> vis_state, int camera_idx)
    : mw::Node("vio_node")
    , vis_state_(std::move(vis_state))
    , camera_idx_(camera_idx) {
  // Build topic names based on camera index (from runner_api.cpp pattern)
  std::string camera_topic = "/camera" + std::to_string(camera_idx) + "/frame";
  std::string camera_info_topic = camera_topic + "/info";
  std::string camera_pose_topic = camera_topic + "/pose";
  std::string imu_topic = "/imu";

  std::cout << "Subscribing to topics:" << std::endl;
  std::cout << "  Image:       " << camera_topic << std::endl;
  std::cout << "  CameraInfo:  " << camera_info_topic << std::endl;
  std::cout << "  CameraPose:  " << camera_pose_topic << std::endl;
  std::cout << "  IMU:         " << imu_topic << std::endl;

  sub_image_ = create_subscription<sensor_msgs::Image>(
    camera_topic,
    [this](const sensor_msgs::Image& msg) { on_image(msg); }
  );

  sub_imu_ = create_subscription<sensor_msgs::Imu>(
    imu_topic,
    [this](const sensor_msgs::Imu& msg) { on_imu(msg); }
  );

  sub_camera_info_ = create_subscription<sensor_msgs::CameraInfo>(
    camera_info_topic,
    [this](const sensor_msgs::CameraInfo& msg) { on_camera_info(msg); }
  );

  sub_camera_pose_ = create_subscription<geometry_msgs::PoseStamped>(
    camera_pose_topic,
    [this](const geometry_msgs::PoseStamped& msg) { on_camera_pose(msg); }
  );

  pub_odom_ = create_publisher<geometry_msgs::PoseStamped>("/vio/odometry");
}

void VioNode::on_camera_info(const sensor_msgs::CameraInfo& msg) {
  if (!vio_.calibration.has_intrinsics) {
    vio_.calibration.set_intrinsics(msg);
  }
}

void VioNode::on_camera_pose(const geometry_msgs::PoseStamped& msg) {
  if (!vio_.calibration.has_extrinsics) {
    vio_.calibration.set_extrinsics(msg);
  }
}

void VioNode::on_imu(const sensor_msgs::Imu& msg) {
  imu_.integrate(msg);
}

void VioNode::on_image(const sensor_msgs::Image& msg) {
  auto now = std::chrono::steady_clock::now();

  cv::Mat frame = from_image_msg(msg);
  if (frame.empty()) {
    return;
  }

  // Convert to grayscale
  cv::Mat gray;
  if (frame.channels() == 3) {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = frame.clone();
  }

  // First frame - just detect features
  if (vio_.prev_gray.empty()) {
    detect_features(gray);
    vio_.prev_gray = gray.clone();
    last_frame_time_ = now;
    first_frame_ = false;

    update_visualization(frame);
    return;
  }

  // Track features using optical flow
  std::vector<cv::Point2f> curr_points;
  std::vector<uchar> status;
  std::vector<float> err;

  if (!vio_.prev_points.empty()) {
    cv::calcOpticalFlowPyrLK(
      vio_.prev_gray,
      gray,
      vio_.prev_points,
      curr_points,
      status,
      err,
      cv::Size(21, 21),
      3,
      cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        30,
        0.01
      )
    );
  }

  // Filter good tracks
  std::vector<cv::Point2f> good_prev, good_curr;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] && err[i] < 12.0) {
      good_prev.push_back(vio_.prev_points[i]);
      good_curr.push_back(curr_points[i]);
    }
  }

  vio_.tracked_count = static_cast<int>(good_curr.size());

  // Estimate motion if we have enough points
  if (good_prev.size() >= 8) {
    cv::Mat E, mask;
    E = cv::findEssentialMat(
      good_prev,
      good_curr,
      vio_.get_camera_matrix(),
      cv::RANSAC,
      0.999,
      1.0,
      mask
    );

    if (!E.empty()) {
      cv::Mat R, t;
      int inliers = cv::recoverPose(
        E,
        good_prev,
        good_curr,
        vio_.get_camera_matrix(),
        R,
        t,
        mask
      );

      if (inliers > 10) {
        // Scale from IMU (use velocity magnitude as proxy)
        double scale = cv::norm(imu_.velocity) * 0.033; // Approximate dt
        if (scale < 0.001)
          scale = 0.05; // Default scale when IMU not available

        // Clamp scale to reasonable values
        scale = std::min(scale, 2.0);
        scale = std::max(scale, 0.01);

        // Update world pose: T_world = T_world * T_delta
        vio_.t_world = vio_.t_world + scale * (vio_.R_world * t);
        vio_.R_world = vio_.R_world * R;

        // Add to trajectory
        cv::Point3d pos(
          vio_.t_world.at<double>(0),
          vio_.t_world.at<double>(1),
          vio_.t_world.at<double>(2)
        );
        vio_.trajectory.push_back(pos);

        // Publish odometry
        double timestamp = get_timestamp_sec(msg);
        auto pose_msg = create_pose_msg(vio_.R_world, vio_.t_world, timestamp);
        pub_odom_.publish(pose_msg);
      }
    }
  }

  // Re-detect features if we're running low
  if (vio_.tracked_count < vio_.min_features) {
    detect_features(gray);
  } else {
    // Keep tracked points
    vio_.prev_points = good_curr;
  }

  // Calculate FPS
  double fps = 0.0;
  if (!first_frame_) {
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_frame_time_
    );
    if (dt.count() > 0) {
      fps = 1000.0 / dt.count();
    }
  }

  // Update visualization state
  {
    std::lock_guard<std::mutex> lock(vis_state_->mutex);
    vis_state_->current_points = good_curr;
    vis_state_->prev_points = good_prev;
    vis_state_->tracking_status = status;
    vis_state_->trajectory_3d = vio_.trajectory;
    vis_state_->current_position = cv::Point3d(
      vio_.t_world.at<double>(0),
      vio_.t_world.at<double>(1),
      vio_.t_world.at<double>(2)
    );
    vis_state_->tracked_features = vio_.tracked_count;
    vis_state_->detected_features = vio_.detected_count;
    vis_state_->fps = fps;
    vis_state_->has_intrinsics = vio_.calibration.has_intrinsics;
    vis_state_->has_extrinsics = vio_.calibration.has_extrinsics;
    vis_state_->fx = vio_.calibration.fx;
    vis_state_->fy = vio_.calibration.fy;
    vis_state_->cx = vio_.calibration.cx;
    vis_state_->cy = vio_.calibration.cy;
    vis_state_->has_new_data = true;
  }

  update_visualization(frame);

  vio_.prev_gray = gray.clone();
  last_frame_time_ = now;
}

void VioNode::detect_features(const cv::Mat& gray) {
  std::vector<cv::KeyPoint> keypoints;
  vio_.orb->detect(gray, keypoints);

  // Sort by response and keep best
  std::sort(
    keypoints.begin(),
    keypoints.end(),
    [](const auto& a, const auto& b) { return a.response > b.response; }
  );

  if (keypoints.size() > static_cast<size_t>(vio_.max_features)) {
    keypoints.resize(vio_.max_features);
  }

  vio_.prev_keypoints = keypoints;
  vio_.prev_points.clear();
  for (const auto& kp : keypoints) {
    vio_.prev_points.push_back(kp.pt);
  }

  vio_.detected_count = static_cast<int>(vio_.prev_points.size());
}

void VioNode::update_visualization(const cv::Mat& frame) {
  cv::Mat vis_frame;
  if (frame.channels() == 1) {
    cv::cvtColor(frame, vis_frame, cv::COLOR_GRAY2BGR);
  } else {
    vis_frame = frame.clone();
  }

  // Draw tracked features
  std::lock_guard<std::mutex> lock(vis_state_->mutex);

  for (size_t i = 0; i < vis_state_->current_points.size(); ++i) {
    cv::circle(
      vis_frame,
      vis_state_->current_points[i],
      4,
      cv::Scalar(0, 255, 0),
      -1
    );

    if (i < vis_state_->prev_points.size()) {
      cv::line(
        vis_frame,
        vis_state_->prev_points[i],
        vis_state_->current_points[i],
        cv::Scalar(0, 255, 255),
        1
      );
    }
  }

  vis_state_->frame = vis_frame;
}

// VioState implementation
VioState::VioState() {
  orb = cv::ORB::create(max_features);
  trajectory.push_back(cv::Point3d(0, 0, 0));
}

cv::Mat VioState::get_camera_matrix() const {
  return calibration.get_camera_matrix();
}

} // namespace jr::vio

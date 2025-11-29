#include "vio/multi_camera_vio_node.hpp"
#include "vio/msg_utils.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <iostream>

namespace jr::vio {

MultiCameraVioNode::MultiCameraVioNode(
  std::shared_ptr<MultiCameraVisState> vis_state
)
    : mw::Node("multi_camera_vio_node")
    , vis_state_(std::move(vis_state)) {
  trajectory_.push_back(cv::Point3d(0, 0, 0));

  std::cout << "=== Multi-Camera VIO Node ===" << std::endl;
  std::cout << "Subscribing to " << MAX_CAMERAS << " cameras..." << std::endl;
  std::cout << "Back cameras for VIO: " << BACK_CAM_WIDE << " (wide), "
            << BACK_CAM_TELE << " (tele)" << std::endl;
  std::cout << "Front camera (display only): " << FRONT_CAMERA << std::endl;

  // Subscribe to all cameras
  for (int i = 0; i < MAX_CAMERAS; ++i) {
    std::string camera_topic = "/camera" + std::to_string(i) + "/frame";
    std::string camera_info_topic = camera_topic + "/info";
    std::string camera_pose_topic = camera_topic + "/pose";

    std::cout << "  Camera " << i;
    if (i == FRONT_CAMERA) {
      std::cout << " (front - display only)";
    } else {
      std::cout << " (back - VIO)";
    }
    std::cout << ":" << std::endl;
    std::cout << "    Image: " << camera_topic << std::endl;

    sub_images_[i] = create_subscription<sensor_msgs::Image>(
      camera_topic,
      [this, i](const sensor_msgs::Image& msg) { on_image(i, msg); }
    );

    sub_camera_infos_[i] = create_subscription<sensor_msgs::CameraInfo>(
      camera_info_topic,
      [this, i](const sensor_msgs::CameraInfo& msg) { on_camera_info(i, msg); }
    );

    sub_camera_poses_[i] = create_subscription<geometry_msgs::PoseStamped>(
      camera_pose_topic,
      [this, i](const geometry_msgs::PoseStamped& msg) {
        on_camera_pose(i, msg);
      }
    );
  }

  // Subscribe to IMU
  sub_imu_ = create_subscription<sensor_msgs::Imu>(
    "/imu",
    [this](const sensor_msgs::Imu& msg) { on_imu(msg); }
  );
  std::cout << "  IMU: /imu" << std::endl;

  // Publisher
  pub_odom_ = create_publisher<geometry_msgs::PoseStamped>("/vio/odometry");
  std::cout << "Publishing to: /vio/odometry" << std::endl;
  std::cout << std::endl;
}

void MultiCameraVioNode::on_camera_info(
  int cam_idx,
  const sensor_msgs::CameraInfo& msg
) {
  if (!cameras_[cam_idx].vio.calibration.has_intrinsics) {
    std::cout << "[Camera " << cam_idx << "] ";
    cameras_[cam_idx].vio.calibration.set_intrinsics(msg);
  }
}

void MultiCameraVioNode::on_camera_pose(
  int cam_idx,
  const geometry_msgs::PoseStamped& msg
) {
  if (!cameras_[cam_idx].vio.calibration.has_extrinsics) {
    std::cout << "[Camera " << cam_idx << "] ";
    cameras_[cam_idx].vio.calibration.set_extrinsics(msg);
  }
}

void MultiCameraVioNode::on_imu(const sensor_msgs::Imu& msg) {
  imu_.integrate(msg);
}

void MultiCameraVioNode::on_image(int cam_idx, const sensor_msgs::Image& msg) {
  auto now = std::chrono::steady_clock::now();
  auto& cam = cameras_[cam_idx];

  cv::Mat frame = from_image_msg(msg);
  if (frame.empty()) {
    return;
  }

  cam.frame = frame.clone();
  cam.active = true;
  cam.frame_timestamp_ns =
    static_cast<uint64_t>(msg.header().stamp().sec()) * 1000000000ULL +
    msg.header().stamp().nsec();

  // Convert to grayscale
  cv::Mat gray;
  if (frame.channels() == 3) {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = frame.clone();
  }
  cam.gray = gray.clone();

  // First frame - just detect features
  if (cam.vio.prev_gray.empty()) {
    detect_features(cam_idx, gray);
    cam.vio.prev_gray = gray.clone();
    cam.last_frame_time = now;
    cam.first_frame = false;
    update_visualization(cam_idx, frame);
    return;
  }

  // Track features using optical flow
  std::vector<cv::Point2f> curr_points;
  std::vector<uchar> status;
  std::vector<float> err;

  if (!cam.vio.prev_points.empty()) {
    cv::calcOpticalFlowPyrLK(
      cam.vio.prev_gray,
      gray,
      cam.vio.prev_points,
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

  // Filter good tracks and compute flow magnitude
  cam.good_prev.clear();
  cam.good_curr.clear();
  cam.flow_magnitude.clear();

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] && err[i] < 12.0) {
      cam.good_prev.push_back(cam.vio.prev_points[i]);
      cam.good_curr.push_back(curr_points[i]);

      // Compute flow magnitude
      float dx = curr_points[i].x - cam.vio.prev_points[i].x;
      float dy = curr_points[i].y - cam.vio.prev_points[i].y;
      cam.flow_magnitude.push_back(std::sqrt(dx * dx + dy * dy));
    }
  }

  cam.vio.tracked_count = static_cast<int>(cam.good_curr.size());

  // Only use back cameras for VIO (skip front camera)
  std::vector<std::pair<int, std::pair<cv::Mat, cv::Mat>>> estimates;

  if (cam_idx != FRONT_CAMERA && cam.good_prev.size() >= 8) {
    cv::Mat E, mask;
    E = cv::findEssentialMat(
      cam.good_prev,
      cam.good_curr,
      cam.vio.get_camera_matrix(),
      cv::RANSAC,
      0.999,
      1.0,
      mask
    );

    if (!E.empty()) {
      cv::Mat R, t;
      int inliers = cv::recoverPose(
        E,
        cam.good_prev,
        cam.good_curr,
        cam.vio.get_camera_matrix(),
        R,
        t,
        mask
      );

      if (inliers > 10) {
        estimates.push_back({cam_idx, {R.clone(), t.clone()}});
      }
    }
  }

  // Fuse motion estimates (only from back cameras)
  if (!estimates.empty()) {
    double timestamp = get_timestamp_sec(msg);
    fuse_motion_estimates(estimates, timestamp);
  }

  // Re-detect features if running low
  if (cam.vio.tracked_count < cam.vio.min_features) {
    detect_features(cam_idx, gray);
  } else {
    cam.vio.prev_points = cam.good_curr;
  }

  // Calculate FPS
  if (!cam.first_frame) {
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - cam.last_frame_time
    );
    if (dt.count() > 0) {
      cam.fps = 1000.0 / dt.count();
    }
  }

  update_visualization(cam_idx, frame);

  // Compute depth from optical flow for back cameras
  // MUST be done BEFORE updating prev_gray!
  if (cam_idx == BACK_CAM_WIDE) {
    frame_counter_++;

    // Compute both sparse and dense depth
    compute_sparse_depth(cam_idx);
    compute_dense_depth(cam_idx);

    // Triangulate points every few frames
    if (frame_counter_ % 5 == 0) {
      triangulate_from_motion(cam_idx);
    }
  }

  // Update previous frame for next iteration
  cam.vio.prev_gray = gray.clone();
  cam.last_frame_time = now;
}

void MultiCameraVioNode::compute_sparse_depth(int cam_idx) {
  auto& cam = cameras_[cam_idx];

  if (cam.good_curr.empty() || cam.flow_magnitude.empty()) {
    return;
  }

  // Create sparse depth visualization from tracked feature flow
  cv::Mat depth_vis = cv::Mat::zeros(cam.gray.size(), CV_8UC3);

  // Find flow statistics
  float max_flow = 0.01f;
  float min_flow = std::numeric_limits<float>::max();
  for (float f : cam.flow_magnitude) {
    if (f > 0.5f) {
      max_flow = std::max(max_flow, f);
      min_flow = std::min(min_flow, f);
    }
  }

  // Draw sparse depth visualization
  for (size_t i = 0; i < cam.good_curr.size(); ++i) {
    float flow = cam.flow_magnitude[i];
    if (flow < 0.5f)
      continue;

    float t =
      std::clamp((flow - min_flow) / (max_flow - min_flow + 0.01f), 0.0f, 1.0f);

    // High flow (close) = red, low flow (far) = blue
    cv::Scalar color(255 * (1 - t), 100, 255 * t);

    int x = static_cast<int>(cam.good_curr[i].x);
    int y = static_cast<int>(cam.good_curr[i].y);

    if (x >= 0 && x < depth_vis.cols && y >= 0 && y < depth_vis.rows) {
      cv::circle(depth_vis, cv::Point(x, y), 5, color, -1);

      // Draw flow vector
      int px = static_cast<int>(cam.good_prev[i].x);
      int py = static_cast<int>(cam.good_prev[i].y);
      cv::line(depth_vis, cv::Point(px, py), cv::Point(x, y), color, 1);
    }
  }

  std::lock_guard<std::mutex> lock(vis_state_->mutex);
  vis_state_->sparse_depth_map = depth_vis;
  vis_state_->min_flow = min_flow;
  vis_state_->max_flow = max_flow;
}

void MultiCameraVioNode::compute_dense_depth(int cam_idx) {
  auto& cam = cameras_[cam_idx];

  if (cam.vio.prev_gray.empty() || cam.gray.empty()) {
    return;
  }

  // Downsample for faster computation
  cv::Mat prev_small, curr_small;
  double scale = 0.5;
  cv::resize(cam.vio.prev_gray, prev_small, cv::Size(), scale, scale);
  cv::resize(cam.gray, curr_small, cv::Size(), scale, scale);

  // Compute dense optical flow using Farneback
  cv::Mat flow;
  cv::calcOpticalFlowFarneback(
    prev_small,
    curr_small,
    flow,
    0.5, // pyr_scale
    3,   // levels
    15,  // winsize
    3,   // iterations
    5,   // poly_n
    1.2, // poly_sigma
    0    // flags
  );

  // Split flow into x and y components
  std::vector<cv::Mat> flow_parts(2);
  cv::split(flow, flow_parts);
  cv::Mat flow_x = flow_parts[0];
  cv::Mat flow_y = flow_parts[1];

  // Compute magnitude and angle
  cv::Mat magnitude, angle;
  cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

  // Find magnitude range for normalization
  double min_mag_d, max_mag_d;
  cv::minMaxLoc(magnitude, &min_mag_d, &max_mag_d);
  float max_mag = static_cast<float>(max_mag_d);

  // Create HSV visualization (hue = direction, value = magnitude)
  cv::Mat hsv(flow.size(), CV_8UC3);

  // Better normalization - use percentile-based scaling
  float mag_threshold = max_mag * 0.05f; // Ignore bottom 5% as noise

  for (int y = 0; y < flow.rows; ++y) {
    for (int x = 0; x < flow.cols; ++x) {
      float mag = magnitude.at<float>(y, x);
      float ang = angle.at<float>(y, x);

      // Hue: flow direction (0-180 for OpenCV HSV)
      uchar h = static_cast<uchar>(ang / 2.0f);

      // Saturation: full
      uchar s = 255;

      // Value: normalized magnitude (higher = brighter)
      float norm_mag = (max_mag > mag_threshold)
                         ? std::clamp(mag / max_mag, 0.0f, 1.0f)
                         : 0.0f;
      uchar v = static_cast<uchar>(norm_mag * 255);

      hsv.at<cv::Vec3b>(y, x) = cv::Vec3b(h, s, v);
    }
  }

  // Convert HSV to BGR
  cv::Mat flow_bgr;
  cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);

  // Upscale back to original size
  cv::Mat flow_bgr_full;
  cv::resize(flow_bgr, flow_bgr_full, cam.gray.size());

  // Create dense depth map (magnitude only, colored by depth)
  cv::Mat depth_dense(flow.size(), CV_8UC3);

  for (int y = 0; y < flow.rows; ++y) {
    for (int x = 0; x < flow.cols; ++x) {
      float mag = magnitude.at<float>(y, x);

      // Normalize: use square root for better dynamic range in visualization
      float t = (max_mag > mag_threshold)
                  ? std::clamp(std::sqrt(mag / max_mag), 0.0f, 1.0f)
                  : 0.0f;

      // Red = close (high flow), Blue = far (low flow)
      uchar r = static_cast<uchar>(t * 255);
      uchar g = static_cast<uchar>(80 + t * 120);
      uchar b = static_cast<uchar>((1.0f - t) * 255);

      depth_dense.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
    }
  }

  cv::Mat depth_dense_full;
  cv::resize(depth_dense, depth_dense_full, cam.gray.size());

  std::lock_guard<std::mutex> lock(vis_state_->mutex);
  vis_state_->dense_depth_map = depth_dense_full;
  vis_state_->flow_hsv = flow_bgr_full;
  vis_state_->has_depth = true;
}

void MultiCameraVioNode::triangulate_from_motion(int cam_idx) {
  auto& cam = cameras_[cam_idx];

  if (cam.good_prev.size() < 8 || cam.good_curr.size() < 8) {
    return;
  }

  if (!cam.vio.calibration.has_intrinsics) {
    return;
  }

  // Use essential matrix to get relative pose between frames
  cv::Mat E, mask;
  E = cv::findEssentialMat(
    cam.good_prev,
    cam.good_curr,
    cam.vio.get_camera_matrix(),
    cv::RANSAC,
    0.999,
    1.0,
    mask
  );

  if (E.empty()) {
    return;
  }

  cv::Mat R_delta, t_delta;
  int inliers = cv::recoverPose(
    E,
    cam.good_prev,
    cam.good_curr,
    cam.vio.get_camera_matrix(),
    R_delta,
    t_delta,
    mask
  );

  if (inliers < 10) {
    return;
  }

  // Create projection matrices for triangulation
  cv::Mat K = cam.vio.get_camera_matrix();

  // P1 = K * [I | 0]
  cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
  K.copyTo(P1(cv::Rect(0, 0, 3, 3)));

  // P2 = K * [R | t]
  cv::Mat Rt = cv::Mat::zeros(3, 4, CV_64F);
  R_delta.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
  t_delta.copyTo(Rt(cv::Rect(3, 0, 1, 3)));
  cv::Mat P2 = K * Rt;

  // Filter points by mask
  std::vector<cv::Point2f> pts_prev, pts_curr;
  std::vector<cv::Vec3b> colors;

  for (size_t i = 0; i < cam.good_prev.size(); ++i) {
    if (mask.at<uchar>(static_cast<int>(i))) {
      pts_prev.push_back(cam.good_prev[i]);
      pts_curr.push_back(cam.good_curr[i]);

      // Get color
      cv::Point pt = cam.good_curr[i];
      if (pt.x >= 0 && pt.x < cam.frame.cols && pt.y >= 0 &&
          pt.y < cam.frame.rows) {
        if (cam.frame.channels() == 3) {
          colors.push_back(cam.frame.at<cv::Vec3b>(pt));
        } else {
          uchar g = cam.frame.at<uchar>(pt);
          colors.push_back(cv::Vec3b(g, g, g));
        }
      } else {
        colors.push_back(cv::Vec3b(128, 128, 128));
      }
    }
  }

  if (pts_prev.size() < 8) {
    return;
  }

  // Triangulate
  cv::Mat points4D;
  cv::triangulatePoints(P1, P2, pts_prev, pts_curr, points4D);

  // Convert to 3D and transform to world frame
  std::vector<ColorPoint3D> new_points;

  std::lock_guard<std::mutex> pose_lock(pose_mutex_);

  // Get scale from IMU
  double scale = cv::norm(imu_.velocity) * 0.033;
  if (scale < 0.01)
    scale = 0.1;
  scale = std::clamp(scale, 0.01, 2.0);

  for (int i = 0; i < points4D.cols; ++i) {
    cv::Mat p = points4D.col(i);
    double w = p.at<float>(3);
    if (std::abs(w) < 1e-6)
      continue;

    cv::Point3d pt_cam(
      p.at<float>(0) / w,
      p.at<float>(1) / w,
      p.at<float>(2) / w
    );

    // Filter by depth (only keep points in front of camera at reasonable
    // distance)
    if (pt_cam.z < 0.1 || pt_cam.z > 30.0)
      continue;

    // Scale the point
    pt_cam *= scale;

    // Transform to world frame
    cv::Mat pt_cam_mat =
      (cv::Mat_<double>(3, 1) << pt_cam.x, pt_cam.y, pt_cam.z);

    // Apply camera extrinsics if available
    cv::Mat pt_imu;
    if (cam.vio.calibration.has_extrinsics) {
      pt_imu = cam.vio.calibration.R_imu_camera * pt_cam_mat +
               cam.vio.calibration.t_imu_camera;
    } else {
      pt_imu = pt_cam_mat;
    }

    // Transform to world
    cv::Mat pt_world = R_world_ * pt_imu + t_world_;

    ColorPoint3D cp;
    cp.position = cv::Point3d(
      pt_world.at<double>(0),
      pt_world.at<double>(1),
      pt_world.at<double>(2)
    );
    cp.color = (i < static_cast<int>(colors.size())) ? colors[i]
                                                     : cv::Vec3b(128, 128, 128);
    cp.confidence = 1.0;

    new_points.push_back(cp);
  }

  add_to_pointcloud(new_points);
}

void MultiCameraVioNode::add_to_pointcloud(
  const std::vector<ColorPoint3D>& new_points
) {
  std::lock_guard<std::mutex> lock(vis_state_->mutex);

  for (const auto& p : new_points) {
    vis_state_->pointcloud.push_back(p);
  }

  // Limit pointcloud size
  while (vis_state_->pointcloud.size() >
         MultiCameraVisState::MAX_POINTCLOUD_SIZE) {
    vis_state_->pointcloud.erase(vis_state_->pointcloud.begin());
  }
}

std::pair<cv::Mat, cv::Mat> MultiCameraVioNode::transform_to_imu_frame(
  int cam_idx,
  const cv::Mat& R_cam,
  const cv::Mat& t_cam
) {
  const auto& calib = cameras_[cam_idx].vio.calibration;

  if (!calib.has_extrinsics) {
    return {R_cam.clone(), t_cam.clone()};
  }

  cv::Mat R_imu = calib.R_imu_camera * R_cam * calib.R_imu_camera.t();
  cv::Mat t_imu = calib.R_imu_camera * t_cam;

  return {R_imu, t_imu};
}

void MultiCameraVioNode::fuse_motion_estimates(
  const std::vector<std::pair<int, std::pair<cv::Mat, cv::Mat>>>& estimates,
  double timestamp
) {
  if (estimates.empty()) {
    return;
  }

  cv::Mat R_fused = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat t_fused = cv::Mat::zeros(3, 1, CV_64F);
  double total_weight = 0.0;

  for (const auto& [cam_idx, Rt] : estimates) {
    // Skip front camera
    if (cam_idx == FRONT_CAMERA) {
      continue;
    }

    const auto& [R_cam, t_cam] = Rt;
    auto [R_imu, t_imu] = transform_to_imu_frame(cam_idx, R_cam, t_cam);

    double weight = static_cast<double>(cameras_[cam_idx].vio.tracked_count);
    weight = std::max(weight, 1.0);

    R_fused += weight * R_imu;
    t_fused += weight * t_imu;
    total_weight += weight;
  }

  if (total_weight > 0) {
    R_fused /= total_weight;
    t_fused /= total_weight;

    // Re-orthogonalize
    cv::SVD svd(R_fused, cv::SVD::FULL_UV);
    R_fused = svd.u * svd.vt;

    if (cv::determinant(R_fused) < 0) {
      R_fused = -R_fused;
    }

    // Scale from IMU
    double scale = cv::norm(imu_.velocity) * 0.033;
    if (scale < 0.001)
      scale = 0.05;
    scale = std::clamp(scale, 0.01, 2.0);

    std::lock_guard<std::mutex> lock(pose_mutex_);
    t_world_ = t_world_ + scale * (R_world_ * t_fused);
    R_world_ = R_world_ * R_fused;

    cv::Point3d pos(
      t_world_.at<double>(0),
      t_world_.at<double>(1),
      t_world_.at<double>(2)
    );
    trajectory_.push_back(pos);

    {
      std::lock_guard<std::mutex> lock2(vis_state_->mutex);
      vis_state_->trajectory_3d = trajectory_;
      vis_state_->current_position = pos;
    }

    auto pose_msg = create_pose_msg(R_world_, t_world_, timestamp);
    pub_odom_.publish(pose_msg);
  }
}

void MultiCameraVioNode::detect_features(int cam_idx, const cv::Mat& gray) {
  auto& cam = cameras_[cam_idx];
  std::vector<cv::KeyPoint> keypoints;
  cam.vio.orb->detect(gray, keypoints);

  std::sort(
    keypoints.begin(),
    keypoints.end(),
    [](const auto& a, const auto& b) { return a.response > b.response; }
  );

  if (keypoints.size() > static_cast<size_t>(cam.vio.max_features)) {
    keypoints.resize(cam.vio.max_features);
  }

  cam.vio.prev_keypoints = keypoints;
  cam.vio.prev_points.clear();
  for (const auto& kp : keypoints) {
    cam.vio.prev_points.push_back(kp.pt);
  }

  cam.vio.detected_count = static_cast<int>(cam.vio.prev_points.size());
}

void MultiCameraVioNode::update_visualization(
  int cam_idx,
  const cv::Mat& frame
) {
  auto& cam = cameras_[cam_idx];

  cv::Mat vis_frame;
  if (frame.channels() == 1) {
    cv::cvtColor(frame, vis_frame, cv::COLOR_GRAY2BGR);
  } else {
    vis_frame = frame.clone();
  }

  // Draw tracked features with flow vectors
  for (size_t i = 0; i < cam.good_curr.size(); ++i) {
    // Color by flow magnitude (if available)
    cv::Scalar color(0, 255, 0);
    if (i < cam.flow_magnitude.size() && cam.flow_magnitude[i] > 0.5f) {
      float t = std::min(cam.flow_magnitude[i] / 20.0f, 1.0f);
      color = cv::Scalar(255 * (1 - t), 100, 255 * t);
    }

    cv::circle(vis_frame, cam.good_curr[i], 4, color, -1);

    if (i < cam.good_prev.size()) {
      cv::line(
        vis_frame,
        cam.good_prev[i],
        cam.good_curr[i],
        cv::Scalar(0, 255, 255),
        1
      );
    }
  }

  // Camera label
  std::string cam_label = "Camera " + std::to_string(cam_idx);
  if (cam_idx == FRONT_CAMERA) {
    cam_label += " (Front)";
  }
  cv::putText(
    vis_frame,
    cam_label,
    cv::Point(10, 25),
    cv::FONT_HERSHEY_SIMPLEX,
    0.7,
    cv::Scalar(255, 255, 255),
    2
  );

  std::lock_guard<std::mutex> lock(vis_state_->mutex);
  vis_state_->frames[cam_idx] = vis_frame;
  vis_state_->frame_valid[cam_idx] = true;
  vis_state_->tracked_features[cam_idx] = cam.vio.tracked_count;
  vis_state_->detected_features[cam_idx] = cam.vio.detected_count;
  vis_state_->fps[cam_idx] = cam.fps;
  vis_state_->has_intrinsics[cam_idx] = cam.vio.calibration.has_intrinsics;
  vis_state_->has_extrinsics[cam_idx] = cam.vio.calibration.has_extrinsics;

  int active = 0;
  for (int i = 0; i < MAX_CAMERAS; ++i) {
    if (cameras_[i].active)
      ++active;
  }
  vis_state_->active_cameras = active;
  vis_state_->has_new_data = true;
}

} // namespace jr::vio

#include "vio/multi_camera_rerun_node.hpp"

#include <middleware/middleware.hpp>

#include <opencv2/imgproc.hpp>

#include <iostream>

namespace jr::vio {

MultiCameraRerunNode::MultiCameraRerunNode(
  std::shared_ptr<MultiCameraVisState> vis_state,
  const std::string& recording_name
)
    : mw::Node("multi_camera_rerun")
    , vis_state_(std::move(vis_state))
    , rec_(recording_name) {
  // Initialize ReRun recording stream
  rec_.spawn().exit_on_failure();

  // Set up 3D view coordinate system
  rec_.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  std::cout << "=== ReRun Visualizer ===" << std::endl;
  std::cout << "Recording: " << recording_name << std::endl;
  std::cout << "Open ReRun viewer to see visualization" << std::endl;
  std::cout << std::endl;
}

void MultiCameraRerunNode::spin_once() {
  std::lock_guard<std::mutex> lock(vis_state_->mutex);

  if (!vis_state_->has_new_data) {
    return;
  }

  // Set frame time
  rec_.set_time_sequence("frame", static_cast<int64_t>(frame_counter_++));

  log_camera_frames();
  log_depth_maps();
  log_pointcloud();
  log_trajectory();

  vis_state_->has_new_data = false;
}

void MultiCameraRerunNode::log_camera_frames() {
  for (int i = 0; i < MAX_CAMERAS; ++i) {
    if (!vis_state_->frame_valid[i] || vis_state_->frames[i].empty()) {
      continue;
    }

    const cv::Mat& frame = vis_state_->frames[i];

    // Convert BGR to RGB for ReRun
    cv::Mat rgb;
    if (frame.channels() == 3) {
      cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    } else if (frame.channels() == 1) {
      cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
    } else {
      rgb = frame.clone();
    }

    // Create camera entity path
    std::string cam_path = "cameras/cam_" + std::to_string(i);
    if (i == FRONT_CAMERA) {
      cam_path = "cameras/cam_" + std::to_string(i) + "_front";
    }

    // Log image - borrow the data with proper size calculation
    size_t num_bytes = static_cast<size_t>(rgb.cols) * static_cast<size_t>(rgb.rows) * 3;
    rec_.log(
      cam_path + "/image",
      rerun::Image::from_rgb24(
        rerun::borrow(rgb.data, num_bytes),
        {static_cast<uint32_t>(rgb.cols), static_cast<uint32_t>(rgb.rows)}
      )
    );

    // Log feature info as annotation
    std::string info = "Tracked: " + std::to_string(vis_state_->tracked_features[i]) +
                       " | Detected: " + std::to_string(vis_state_->detected_features[i]) +
                       " | FPS: " + std::to_string(static_cast<int>(vis_state_->fps[i]));
    rec_.log(
      cam_path + "/info",
      rerun::TextLog(info)
    );
  }
}

void MultiCameraRerunNode::log_depth_maps() {
  if (!vis_state_->has_depth) {
    return;
  }

  // Log sparse depth map
  if (!vis_state_->sparse_depth_map.empty()) {
    cv::Mat rgb;
    cv::cvtColor(vis_state_->sparse_depth_map, rgb, cv::COLOR_BGR2RGB);
    size_t num_bytes = static_cast<size_t>(rgb.cols) * static_cast<size_t>(rgb.rows) * 3;

    rec_.log(
      "depth/sparse",
      rerun::Image::from_rgb24(
        rerun::borrow(rgb.data, num_bytes),
        {static_cast<uint32_t>(rgb.cols), static_cast<uint32_t>(rgb.rows)}
      )
    );
  }

  // Log dense depth map
  if (!vis_state_->dense_depth_map.empty()) {
    cv::Mat rgb;
    cv::cvtColor(vis_state_->dense_depth_map, rgb, cv::COLOR_BGR2RGB);
    size_t num_bytes = static_cast<size_t>(rgb.cols) * static_cast<size_t>(rgb.rows) * 3;

    rec_.log(
      "depth/dense",
      rerun::Image::from_rgb24(
        rerun::borrow(rgb.data, num_bytes),
        {static_cast<uint32_t>(rgb.cols), static_cast<uint32_t>(rgb.rows)}
      )
    );
  }

  // Log optical flow HSV visualization
  if (!vis_state_->flow_hsv.empty()) {
    cv::Mat rgb;
    cv::cvtColor(vis_state_->flow_hsv, rgb, cv::COLOR_BGR2RGB);
    size_t num_bytes = static_cast<size_t>(rgb.cols) * static_cast<size_t>(rgb.rows) * 3;

    rec_.log(
      "depth/flow",
      rerun::Image::from_rgb24(
        rerun::borrow(rgb.data, num_bytes),
        {static_cast<uint32_t>(rgb.cols), static_cast<uint32_t>(rgb.rows)}
      )
    );
  }

  // Log flow statistics
  rec_.log(
    "depth/stats",
    rerun::TextLog(
      "Flow range: " + std::to_string(vis_state_->min_flow) +
      " - " + std::to_string(vis_state_->max_flow) + " px"
    )
  );
}

void MultiCameraRerunNode::log_pointcloud() {
  if (vis_state_->pointcloud.empty()) {
    return;
  }

  // Extract positions and colors
  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;
  positions.reserve(vis_state_->pointcloud.size());
  colors.reserve(vis_state_->pointcloud.size());

  for (const auto& p : vis_state_->pointcloud) {
    positions.push_back(rerun::Position3D(
      static_cast<float>(p.position.x),
      static_cast<float>(p.position.y),
      static_cast<float>(p.position.z)
    ));
    colors.push_back(rerun::Color(p.color[2], p.color[1], p.color[0])); // BGR to RGB
  }

  rec_.log(
    "world/pointcloud",
    rerun::Points3D(positions)
      .with_colors(colors)
      .with_radii({0.08f})
  );

  // Log pointcloud count
  rec_.log(
    "world/pointcloud/count",
    rerun::TextLog("Points: " + std::to_string(vis_state_->pointcloud.size()))
  );
}

void MultiCameraRerunNode::log_trajectory() {
  const auto& traj = vis_state_->trajectory_3d;
  const auto& pos = vis_state_->current_position;

  // Log current position as a point
  rec_.log(
    "world/camera",
    rerun::Points3D({{
      static_cast<float>(pos.x),
      static_cast<float>(pos.y),
      static_cast<float>(pos.z)
    }})
      .with_colors({rerun::Color(0, 255, 255)}) // Cyan
      .with_radii({0.2f})
  );

  // Log position text
  char pos_str[64];
  snprintf(pos_str, sizeof(pos_str), "Pos: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);
  rec_.log("world/camera/position", rerun::TextLog(pos_str));

  // Log trajectory as a line strip
  if (traj.size() >= 2) {
    std::vector<rerun::Position3D> line_points;
    std::vector<rerun::Color> line_colors;
    line_points.reserve(traj.size());
    line_colors.reserve(traj.size());

    for (size_t i = 0; i < traj.size(); ++i) {
      line_points.push_back(rerun::Position3D(
        static_cast<float>(traj[i].x),
        static_cast<float>(traj[i].y),
        static_cast<float>(traj[i].z)
      ));

      // Color gradient: blue (old) -> red (new)
      float t = static_cast<float>(i) / static_cast<float>(traj.size());
      line_colors.push_back(rerun::Color(
        static_cast<uint8_t>(255 * t),
        100,
        static_cast<uint8_t>(255 * (1.0f - t))
      ));
    }

    rec_.log(
      "world/trajectory",
      rerun::LineStrips3D({line_points})
        .with_colors({rerun::Color(255, 165, 0)}) // Orange trajectory
        .with_radii({0.05f})
    );

    // Also log trajectory points for better visibility
    rec_.log(
      "world/trajectory_points",
      rerun::Points3D(line_points)
        .with_colors(line_colors)
        .with_radii({0.1f})
    );
  }

  prev_trajectory_size_ = traj.size();

  // Log active cameras count as a scalar
  rec_.log(
    "stats/active_cameras",
    rerun::Scalars(static_cast<double>(vis_state_->active_cameras))
  );
}

} // namespace jr::vio

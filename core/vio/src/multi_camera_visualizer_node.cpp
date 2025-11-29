#include "vio/multi_camera_visualizer_node.hpp"

#include <middleware/middleware.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace jr::vio {

MultiCameraVisualizerNode::MultiCameraVisualizerNode(
  std::shared_ptr<MultiCameraVisState> vis_state
)
    : mw::Node("multi_camera_visualizer")
    , vis_state_(std::move(vis_state))
    , trajectory_img_(300, 300, CV_8UC3, cv::Scalar(30, 30, 30)) {
  reset_trajectory_image();
}

void MultiCameraVisualizerNode::reset_trajectory_image() {
  trajectory_img_ = cv::Mat(300, 300, CV_8UC3, cv::Scalar(30, 30, 30));

  // Draw grid
  for (int i = 0; i < 300; i += 30) {
    cv::line(
      trajectory_img_,
      cv::Point(i, 0),
      cv::Point(i, 300),
      cv::Scalar(50, 50, 50),
      1
    );
    cv::line(
      trajectory_img_,
      cv::Point(0, i),
      cv::Point(300, i),
      cv::Scalar(50, 50, 50),
      1
    );
  }

  // Mark origin
  cv::circle(trajectory_img_, cv::Point(150, 150), 4, cv::Scalar(255, 255, 255), -1);
}

void MultiCameraVisualizerNode::spin_once() {
  std::lock_guard<std::mutex> lock(vis_state_->mutex);

  // Target sizes for each panel
  const int cam_w = 320;
  const int cam_h = 240;
  const int depth_w = 320;
  const int depth_h = 240;
  const int pc_size = 300;
  const int traj_size = 300;

  // Build camera row (3 cameras)
  std::vector<cv::Mat> cam_frames;
  for (int i = 0; i < MAX_CAMERAS; ++i) {
    cv::Mat cam_panel(cam_h, cam_w, CV_8UC3, cv::Scalar(30, 30, 30));

    if (vis_state_->frame_valid[i] && !vis_state_->frames[i].empty()) {
      cv::Mat resized;
      cv::resize(vis_state_->frames[i], resized, cv::Size(cam_w, cam_h));
      resized.copyTo(cam_panel);
    }

    // Add camera info overlay
    std::string cam_label = "Cam " + std::to_string(i);
    if (i == FRONT_CAMERA) {
      cam_label += " (Front)";
    }
    cv::putText(
      cam_panel,
      cam_label,
      cv::Point(5, 18),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(255, 255, 255),
      1
    );

    std::string info = cv::format(
      "Trk:%d Det:%d FPS:%.1f",
      vis_state_->tracked_features[i],
      vis_state_->detected_features[i],
      vis_state_->fps[i]
    );
    cv::putText(
      cam_panel,
      info,
      cv::Point(5, cam_h - 8),
      cv::FONT_HERSHEY_SIMPLEX,
      0.4,
      cv::Scalar(0, 255, 0),
      1
    );

    cam_frames.push_back(cam_panel);
  }

  cv::Mat cam_row;
  cv::hconcat(cam_frames, cam_row);

  // Build depth panels row: sparse depth | dense depth | flow HSV
  cv::Mat sparse_panel(depth_h, depth_w, CV_8UC3, cv::Scalar(30, 30, 30));
  cv::Mat dense_panel(depth_h, depth_w, CV_8UC3, cv::Scalar(30, 30, 30));
  cv::Mat flow_panel(depth_h, depth_w, CV_8UC3, cv::Scalar(30, 30, 30));

  if (vis_state_->has_depth) {
    if (!vis_state_->sparse_depth_map.empty()) {
      cv::Mat resized;
      cv::resize(vis_state_->sparse_depth_map, resized, cv::Size(depth_w, depth_h));
      resized.copyTo(sparse_panel);
    }

    if (!vis_state_->dense_depth_map.empty()) {
      cv::Mat resized;
      cv::resize(vis_state_->dense_depth_map, resized, cv::Size(depth_w, depth_h));
      resized.copyTo(dense_panel);
    }

    if (!vis_state_->flow_hsv.empty()) {
      cv::Mat resized;
      cv::resize(vis_state_->flow_hsv, resized, cv::Size(depth_w, depth_h));
      resized.copyTo(flow_panel);
    }
  }

  // Add labels to depth panels
  cv::putText(
    sparse_panel,
    "Sparse Depth (Features)",
    cv::Point(5, 18),
    cv::FONT_HERSHEY_SIMPLEX,
    0.45,
    cv::Scalar(200, 200, 200),
    1
  );
  // Color legend for sparse
  for (int i = 0; i < 60; ++i) {
    double t = i / 59.0;
    cv::Scalar color(255 * (1 - t), 100, 255 * t);
    cv::line(sparse_panel, cv::Point(200 + i, 8), cv::Point(200 + i, 18), color, 1);
  }
  cv::putText(sparse_panel, "Near", cv::Point(165, 16), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
  cv::putText(sparse_panel, "Far", cv::Point(262, 16), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);

  cv::putText(
    dense_panel,
    "Dense Depth (Farneback)",
    cv::Point(5, 18),
    cv::FONT_HERSHEY_SIMPLEX,
    0.45,
    cv::Scalar(200, 200, 200),
    1
  );
  // Color legend for dense
  for (int i = 0; i < 60; ++i) {
    double t = i / 59.0;
    uchar r = static_cast<uchar>(t * 255);
    uchar g = static_cast<uchar>(50 + t * 100);
    uchar b = static_cast<uchar>((1.0 - t) * 255);
    cv::line(dense_panel, cv::Point(200 + i, 8), cv::Point(200 + i, 18), cv::Scalar(b, g, r), 1);
  }
  cv::putText(dense_panel, "Near", cv::Point(165, 16), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
  cv::putText(dense_panel, "Far", cv::Point(262, 16), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);

  cv::putText(
    flow_panel,
    "Optical Flow (HSV)",
    cv::Point(5, 18),
    cv::FONT_HERSHEY_SIMPLEX,
    0.45,
    cv::Scalar(200, 200, 200),
    1
  );
  cv::putText(
    flow_panel,
    "Hue=Dir, Bright=Speed",
    cv::Point(5, depth_h - 8),
    cv::FONT_HERSHEY_SIMPLEX,
    0.35,
    cv::Scalar(150, 150, 150),
    1
  );

  cv::Mat depth_row;
  cv::hconcat(std::vector<cv::Mat>{sparse_panel, dense_panel, flow_panel}, depth_row);

  // Build pointcloud panel (top-down view)
  cv::Mat pc_panel(pc_size, pc_size, CV_8UC3, cv::Scalar(20, 20, 20));

  // Draw grid
  for (int i = 0; i < pc_size; i += 30) {
    cv::line(pc_panel, cv::Point(i, 0), cv::Point(i, pc_size), cv::Scalar(40, 40, 40), 1);
    cv::line(pc_panel, cv::Point(0, i), cv::Point(pc_size, i), cv::Scalar(40, 40, 40), 1);
  }

  // Draw axes
  int cx = pc_size / 2, cy = pc_size / 2;
  cv::line(pc_panel, cv::Point(cx, cy), cv::Point(cx + 30, cy), cv::Scalar(0, 0, 255), 2); // X red
  cv::line(pc_panel, cv::Point(cx, cy), cv::Point(cx, cy - 30), cv::Scalar(255, 0, 0), 2); // Z blue

  // Draw points
  double scale = 15.0;
  if (!vis_state_->pointcloud.empty()) {
    size_t step = std::max(size_t(1), vis_state_->pointcloud.size() / 3000);
    for (size_t i = 0; i < vis_state_->pointcloud.size(); i += step) {
      const auto& p = vis_state_->pointcloud[i];
      int px = cx + static_cast<int>(p.position.x * scale);
      int py = cy - static_cast<int>(p.position.z * scale);

      if (px >= 0 && px < pc_size && py >= 0 && py < pc_size) {
        cv::Scalar color(p.color[0], p.color[1], p.color[2]);
        cv::circle(pc_panel, cv::Point(px, py), 1, color, -1);
      }
    }
  }

  // Draw camera position
  cv::Point cam_pos(
    cx + static_cast<int>(vis_state_->current_position.x * scale),
    cy - static_cast<int>(vis_state_->current_position.z * scale)
  );
  cam_pos.x = std::clamp(cam_pos.x, 0, pc_size - 1);
  cam_pos.y = std::clamp(cam_pos.y, 0, pc_size - 1);
  cv::circle(pc_panel, cam_pos, 5, cv::Scalar(0, 255, 255), -1);

  cv::putText(
    pc_panel,
    "Pointcloud (Top-Down)",
    cv::Point(5, 18),
    cv::FONT_HERSHEY_SIMPLEX,
    0.45,
    cv::Scalar(200, 200, 200),
    1
  );
  cv::putText(
    pc_panel,
    cv::format("Points: %zu", vis_state_->pointcloud.size()),
    cv::Point(5, 35),
    cv::FONT_HERSHEY_SIMPLEX,
    0.35,
    cv::Scalar(150, 150, 150),
    1
  );

  // Build trajectory panel
  if (vis_state_->has_new_data && !vis_state_->trajectory_3d.empty()) {
    if (vis_state_->trajectory_3d.size() % 500 == 0) {
      reset_trajectory_image();
    }

    double traj_scale = 8.0;
    int tcx = 150, tcy = 150;

    for (size_t i = 1; i < vis_state_->trajectory_3d.size(); ++i) {
      const auto& p1 = vis_state_->trajectory_3d[i - 1];
      const auto& p2 = vis_state_->trajectory_3d[i];

      cv::Point pt1(
        tcx + static_cast<int>(p1.x * traj_scale),
        tcy - static_cast<int>(p1.z * traj_scale)
      );
      cv::Point pt2(
        tcx + static_cast<int>(p2.x * traj_scale),
        tcy - static_cast<int>(p2.z * traj_scale)
      );

      pt1.x = std::clamp(pt1.x, 0, 299);
      pt1.y = std::clamp(pt1.y, 0, 299);
      pt2.x = std::clamp(pt2.x, 0, 299);
      pt2.y = std::clamp(pt2.y, 0, 299);

      double t = static_cast<double>(i) / vis_state_->trajectory_3d.size();
      cv::Scalar color(255 * (1 - t), 100, 255 * t);

      cv::line(trajectory_img_, pt1, pt2, color, 2);
    }

    const auto& curr = vis_state_->current_position;
    cv::Point curr_pt(
      tcx + static_cast<int>(curr.x * traj_scale),
      tcy - static_cast<int>(curr.z * traj_scale)
    );
    curr_pt.x = std::clamp(curr_pt.x, 0, 299);
    curr_pt.y = std::clamp(curr_pt.y, 0, 299);
    cv::circle(trajectory_img_, curr_pt, 5, cv::Scalar(0, 255, 0), -1);

    vis_state_->has_new_data = false;
  }

  cv::Mat traj_panel = trajectory_img_.clone();
  cv::putText(
    traj_panel,
    "Trajectory (Top-Down)",
    cv::Point(5, 18),
    cv::FONT_HERSHEY_SIMPLEX,
    0.45,
    cv::Scalar(200, 200, 200),
    1
  );
  cv::putText(
    traj_panel,
    cv::format("Pos: (%.1f, %.1f, %.1f)",
      vis_state_->current_position.x,
      vis_state_->current_position.y,
      vis_state_->current_position.z),
    cv::Point(5, 35),
    cv::FONT_HERSHEY_SIMPLEX,
    0.35,
    cv::Scalar(150, 150, 150),
    1
  );

  // Combine pointcloud and trajectory side by side
  cv::Mat bottom_left;
  cv::hconcat(pc_panel, traj_panel, bottom_left);

  // Resize bottom_left to match depth_row width
  int depth_row_width = depth_row.cols;
  int bottom_left_width = bottom_left.cols;
  int bottom_left_height = bottom_left.rows;

  // Pad bottom_left to match depth_row width
  if (bottom_left_width < depth_row_width) {
    cv::Mat padding(
      bottom_left_height,
      depth_row_width - bottom_left_width,
      CV_8UC3,
      cv::Scalar(30, 30, 30)
    );

    // Add info panel in the padding
    cv::putText(
      padding,
      "Multi-Camera VIO",
      cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX,
      0.7,
      cv::Scalar(0, 255, 255),
      2
    );
    cv::putText(
      padding,
      cv::format("Active Cameras: %d/%d", vis_state_->active_cameras, MAX_CAMERAS),
      cv::Point(10, 60),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(200, 200, 200),
      1
    );
    cv::putText(
      padding,
      cv::format("Pointcloud: %zu pts", vis_state_->pointcloud.size()),
      cv::Point(10, 85),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(200, 200, 200),
      1
    );
    cv::putText(
      padding,
      cv::format("Flow Range: %.1f - %.1f px",
        vis_state_->min_flow,
        vis_state_->max_flow),
      cv::Point(10, 110),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(200, 200, 200),
      1
    );

    // Controls
    cv::putText(
      padding,
      "Controls:",
      cv::Point(10, 160),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(150, 150, 150),
      1
    );
    cv::putText(
      padding,
      "q/ESC - Quit",
      cv::Point(20, 185),
      cv::FONT_HERSHEY_SIMPLEX,
      0.4,
      cv::Scalar(120, 120, 120),
      1
    );
    cv::putText(
      padding,
      "r - Reset view",
      cv::Point(20, 205),
      cv::FONT_HERSHEY_SIMPLEX,
      0.4,
      cv::Scalar(120, 120, 120),
      1
    );

    // Legend for depth colors
    cv::putText(
      padding,
      "Depth Legend:",
      cv::Point(10, 245),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(150, 150, 150),
      1
    );
    for (int i = 0; i < 100; ++i) {
      double t = i / 99.0;
      cv::Scalar color(255 * (1 - t), 100, 255 * t);
      cv::line(padding, cv::Point(20 + i, 260), cv::Point(20 + i, 280), color, 1);
    }
    cv::putText(padding, "Near", cv::Point(20, bottom_left_height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 255), 1);
    cv::putText(padding, "Far", cv::Point(100, bottom_left_height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 0, 0), 1);

    cv::hconcat(bottom_left, padding, bottom_left);
  }

  // Stack everything vertically: cameras | depth | (pointcloud + trajectory + info)
  cv::Mat display;
  cv::vconcat(cam_row, depth_row, display);
  cv::vconcat(display, bottom_left, display);

  cv::imshow("Multi-Camera VIO", display);

  int key = cv::waitKey(1);
  if (key == 'q' || key == 27) {
    mw::shutdown();
  } else if (key == 'r') {
    reset_trajectory_image();
    vis_state_->pointcloud.clear();
  }
}

} // namespace jr::vio

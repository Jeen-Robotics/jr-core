#include "vio/visualizer_node.hpp"

#include <middleware/middleware.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace jr::vio {

VisualizerNode::VisualizerNode(std::shared_ptr<VisState> vis_state)
    : mw::Node("visualizer")
    , vis_state_(std::move(vis_state))
    , trajectory_img_(600, 600, CV_8UC3, cv::Scalar(30, 30, 30)) {
  reset_trajectory_image();
}

void VisualizerNode::reset_trajectory_image() {
  trajectory_img_ = cv::Mat(600, 600, CV_8UC3, cv::Scalar(30, 30, 30));

  // Draw grid
  for (int i = 0; i < 600; i += 50) {
    cv::line(
      trajectory_img_,
      cv::Point(i, 0),
      cv::Point(i, 600),
      cv::Scalar(50, 50, 50),
      1
    );
    cv::line(
      trajectory_img_,
      cv::Point(0, i),
      cv::Point(600, i),
      cv::Scalar(50, 50, 50),
      1
    );
  }

  // Mark origin
  cv::circle(
    trajectory_img_,
    cv::Point(300, 300),
    5,
    cv::Scalar(255, 255, 255),
    -1
  );
}

void VisualizerNode::spin_once() {
  std::lock_guard<std::mutex> lock(vis_state_->mutex);

  if (!vis_state_->frame.empty()) {
    // Draw info overlay
    cv::Mat display = vis_state_->frame.clone();

    std::string info = cv::format(
      "Features: %d tracked, %d detected | FPS: %.1f",
      vis_state_->tracked_features,
      vis_state_->detected_features,
      vis_state_->fps
    );
    cv::putText(
      display,
      info,
      cv::Point(10, 25),
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(0, 255, 0),
      2
    );

    std::string pos_info = cv::format(
      "Pos: (%.2f, %.2f, %.2f)",
      vis_state_->current_position.x,
      vis_state_->current_position.y,
      vis_state_->current_position.z
    );
    cv::putText(
      display,
      pos_info,
      cv::Point(10, 50),
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(0, 255, 255),
      2
    );

    // Display calibration status
    std::string calib_status = cv::format(
      "Calib: %s | Extr: %s",
      vis_state_->has_intrinsics ? "OK" : "NO",
      vis_state_->has_extrinsics ? "OK" : "NO"
    );
    cv::putText(
      display,
      calib_status,
      cv::Point(10, 75),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      vis_state_->has_intrinsics ? cv::Scalar(0, 255, 0)
                                 : cv::Scalar(0, 165, 255),
      1
    );

    if (vis_state_->has_intrinsics) {
      std::string intrinsics_info = cv::format(
        "fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
        vis_state_->fx,
        vis_state_->fy,
        vis_state_->cx,
        vis_state_->cy
      );
      cv::putText(
        display,
        intrinsics_info,
        cv::Point(10, 95),
        cv::FONT_HERSHEY_SIMPLEX,
        0.4,
        cv::Scalar(150, 150, 150),
        1
      );
    }

    cv::imshow("VIO - Feature Tracking", display);
  }

  // Update trajectory visualization
  if (vis_state_->has_new_data && !vis_state_->trajectory_3d.empty()) {
    // Reset trajectory image periodically if trajectory gets too large
    if (vis_state_->trajectory_3d.size() % 1000 == 0) {
      reset_trajectory_image();
    }

    // Draw trajectory (XZ plane - top-down view)
    double scale = 10.0; // meters to pixels
    int cx = 300, cy = 300;

    for (size_t i = 1; i < vis_state_->trajectory_3d.size(); ++i) {
      const auto& p1 = vis_state_->trajectory_3d[i - 1];
      const auto& p2 = vis_state_->trajectory_3d[i];

      cv::Point pt1(
        cx + static_cast<int>(p1.x * scale),
        cy - static_cast<int>(p1.z * scale)
      );
      cv::Point pt2(
        cx + static_cast<int>(p2.x * scale),
        cy - static_cast<int>(p2.z * scale)
      );

      // Clamp to image bounds
      pt1.x = std::max(0, std::min(599, pt1.x));
      pt1.y = std::max(0, std::min(599, pt1.y));
      pt2.x = std::max(0, std::min(599, pt2.x));
      pt2.y = std::max(0, std::min(599, pt2.y));

      // Color gradient from blue (old) to red (new)
      double t = static_cast<double>(i) / vis_state_->trajectory_3d.size();
      cv::Scalar color(255 * (1 - t), 100, 255 * t);

      cv::line(trajectory_img_, pt1, pt2, color, 2);
    }

    // Draw current position
    const auto& curr = vis_state_->current_position;
    cv::Point curr_pt(
      cx + static_cast<int>(curr.x * scale),
      cy - static_cast<int>(curr.z * scale)
    );
    curr_pt.x = std::max(0, std::min(599, curr_pt.x));
    curr_pt.y = std::max(0, std::min(599, curr_pt.y));
    cv::circle(trajectory_img_, curr_pt, 6, cv::Scalar(0, 255, 0), -1);

    // Draw legend
    cv::putText(
      trajectory_img_,
      "Trajectory (Top-Down XZ View)",
      cv::Point(10, 25),
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(200, 200, 200),
      1
    );
    cv::putText(
      trajectory_img_,
      cv::format("Scale: %.1f m/px", 1.0 / scale),
      cv::Point(10, 50),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(150, 150, 150),
      1
    );

    vis_state_->has_new_data = false;
  }

  cv::imshow("VIO - Trajectory", trajectory_img_);

  int key = cv::waitKey(1);
  if (key == 'q' || key == 27) { // 'q' or ESC
    mw::shutdown();
  } else if (key == 'r') {
    reset_trajectory_image();
  }
}

} // namespace jr::vio

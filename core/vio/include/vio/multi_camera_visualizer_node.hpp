#pragma once

#include "vio/multi_camera_vio_node.hpp"

#include <middleware/node.hpp>

#include <opencv2/core.hpp>

#include <memory>

namespace jr::vio {

/**
 * Multi-Camera Visualizer Node for VIO.
 *
 * Displays feature tracking for all cameras and combined trajectory.
 * Must run in the main thread due to cv::imshow requirements.
 */
class MultiCameraVisualizerNode final : public mw::Node {
public:
  explicit MultiCameraVisualizerNode(
    std::shared_ptr<MultiCameraVisState> vis_state
  );

  void spin_once() override;

private:
  std::shared_ptr<MultiCameraVisState> vis_state_;
  cv::Mat trajectory_img_;

  void reset_trajectory_image();
};

} // namespace jr::vio

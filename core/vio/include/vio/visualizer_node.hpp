#pragma once

#include "vio/vis_state.hpp"

#include <middleware/node.hpp>

#include <opencv2/core.hpp>

#include <memory>

namespace jr::vio {

/**
 * Visualizer Node for VIO.
 *
 * Displays feature tracking and trajectory visualization.
 * Must run in the main thread due to cv::imshow requirements.
 */
class VisualizerNode final : public mw::Node {
public:
  explicit VisualizerNode(std::shared_ptr<VisState> vis_state);

  void spin_once() override;

private:
  std::shared_ptr<VisState> vis_state_;
  cv::Mat trajectory_img_;

  void reset_trajectory_image();
};

} // namespace jr::vio

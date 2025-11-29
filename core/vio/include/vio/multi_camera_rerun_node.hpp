#pragma once

#include "vio/multi_camera_vio_node.hpp"

#include <middleware/node.hpp>

#include <rerun.hpp>

#include <memory>
#include <string>
#include <vector>

namespace jr::vio {

/**
 * Multi-Camera ReRun Visualizer Node for VIO.
 *
 * Visualizes camera frames, tracked features, depth maps,
 * pointcloud, and trajectory using ReRun.
 */
class MultiCameraRerunNode final : public mw::Node {
public:
  explicit MultiCameraRerunNode(
    std::shared_ptr<MultiCameraVisState> vis_state,
    const std::string& recording_name = "jr_vio"
  );

  ~MultiCameraRerunNode() override = default;

  void spin_once() override;

private:
  std::shared_ptr<MultiCameraVisState> vis_state_;
  rerun::RecordingStream rec_;

  // Previous trajectory size for incremental updates
  size_t prev_trajectory_size_ = 0;

  // Frame counter for logging timestamps
  uint64_t frame_counter_ = 0;

  // Log camera frames with features
  void log_camera_frames();

  // Log depth visualizations
  void log_depth_maps();

  // Log pointcloud
  void log_pointcloud();

  // Log trajectory and current pose
  void log_trajectory();
};

} // namespace jr::vio


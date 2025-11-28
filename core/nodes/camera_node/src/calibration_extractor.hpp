#pragma once

#include <geometry_msgs.pb.h>
#include <sensor_msgs.pb.h>

#include <optional>

namespace jr {

class CalibrationExtractor {
public:
  explicit CalibrationExtractor(int camera_idx);

  // Extract calibration info for the given image dimensions.
  // Returns std::nullopt if extraction fails or is not supported on the
  // platform.
  std::optional<sensor_msgs::CameraInfo> extract(int width, int height) const;

  // Extract camera pose.
  // Returns std::nullopt if extraction fails or is not supported on the
  // platform.
  std::optional<geometry_msgs::PoseStamped> extract_pose() const;

private:
  int camera_idx_;
};

} // namespace jr

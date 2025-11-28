#pragma once

#include <geometry_msgs.pb.h>
#include <sensor_msgs.pb.h>

#include <middleware/node.hpp>

namespace jr {

class CameraNode final : public mw::Node {
public:
  struct Config {
    std::string topic_name;
    int camera_idx = 0;
    int delay_ms = 50;
  };

  explicit CameraNode(const std::string& node_name, const Config& config);

  CameraNode(
    const std::string& node_name,
    const Config& config,
    const std::shared_ptr<mw::Middleware>& mw
  );

  ~CameraNode() override;

  void spin_once() override;

  bool valid() const noexcept override;

private:
  Config config_;

  mw::Publisher<sensor_msgs::Image> pub_;
  mw::Publisher<sensor_msgs::CameraInfo> info_pub_;
  mw::Publisher<geometry_msgs::PoseStamped> pose_pub_;

  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

} // namespace jr
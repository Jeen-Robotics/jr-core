#pragma once

#include <sensor_msgs.pb.h>

#include <middleware/node.hpp>

namespace jr {

class ImuNode final : public mw::Node {
public:
  struct Config {
    std::string topic_name = "/imu";
    int delay_ms = 10; // 100 Hz default
  };

  explicit ImuNode(const std::string& node_name, const Config& config);

  ImuNode(
    const std::string& node_name,
    const Config& config,
    const std::shared_ptr<mw::Middleware>& mw
  );

  ~ImuNode() override;

  void spin_once() override;

  bool valid() const noexcept override;

private:
  Config config_;

  mw::Publisher<sensor_msgs::Imu> pub_;

  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

} // namespace jr

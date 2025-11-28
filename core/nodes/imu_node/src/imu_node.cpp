#include "imu_node/imu_node.hpp"

#include <chrono>
#include <thread>

#ifdef __ANDROID__
#include <jr_android/imu_device.hpp>
#endif

namespace jr {

namespace {

sensor_msgs::Imu create_imu_msg(
  double ax,
  double ay,
  double az,
  double gx,
  double gy,
  double gz,
  uint64_t timestamp_ns
) {
  const auto now_sec = static_cast<double>(timestamp_ns) * 1e-9;

  sensor_msgs::Imu msg;

  auto* header = msg.mutable_header();
  header->mutable_stamp()->set_sec(static_cast<int32_t>(now_sec));
  header->mutable_stamp()->set_nsec(
    static_cast<int32_t>(timestamp_ns % 1000000000)
  );
  header->set_frame_id("imu");
  header->set_seq(0);

  // Linear acceleration (m/s^2)
  auto* linear_accel = msg.mutable_linear_acceleration();
  linear_accel->set_x(ax);
  linear_accel->set_y(ay);
  linear_accel->set_z(az);

  // Angular velocity (rad/s)
  auto* angular_vel = msg.mutable_angular_velocity();
  angular_vel->set_x(gx);
  angular_vel->set_y(gy);
  angular_vel->set_z(gz);

  // Set covariance to unknown (all zeros means covariance unknown)
  // sensor_msgs::Imu typically has 9 elements for each covariance matrix
  for (int i = 0; i < 9; ++i) {
    msg.add_orientation_covariance(0.0);
    msg.add_angular_velocity_covariance(0.0);
    msg.add_linear_acceleration_covariance(0.0);
  }

  // Mark orientation as unknown (first element = -1 indicates no orientation
  // data)
  msg.set_orientation_covariance(0, -1.0);

  return msg;
}

} // namespace

#ifdef __ANDROID__

class ImuNode::Impl {
public:
  Impl() {
    imu_device_.setImuCallback([this](const android::ImuData& data) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_data_ = data;
      has_data_ = true;
    });
    valid_ = imu_device_.start();
  }

  ~Impl() {
    imu_device_.stop();
  }

  bool valid() const {
    return valid_;
  }

  void spin_once(const mw::Publisher<sensor_msgs::Imu>& pub) {
    if (!valid_) {
      return;
    }

    android::ImuData data;
    bool has_data = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_data_) {
        data = latest_data_;
        has_data = true;
        has_data_ = false;
      }
    }

    if (has_data) {
      auto msg = create_imu_msg(
        data.ax,
        data.ay,
        data.az,
        data.gx,
        data.gy,
        data.gz,
        data.timestamp_ns
      );
      pub.publish(msg);
    }
  }

private:
  android::ImuDevice imu_device_;
  std::mutex mutex_;
  android::ImuData latest_data_{};
  bool has_data_ = false;
  bool valid_ = false;
};

#else

// Stub implementation for non-Android platforms
class ImuNode::Impl {
public:
  Impl() = default;
  ~Impl() = default;

  bool valid() const {
    return false; // IMU not available on this platform
  }

  void spin_once(const mw::Publisher<sensor_msgs::Imu>& /*pub*/) {
    // No IMU data available on non-Android platforms
  }
};

#endif

ImuNode::ImuNode(const std::string& node_name, const Config& config)
    : ImuNode(node_name, config, mw::get()) {
}

ImuNode::ImuNode(
  const std::string& node_name,
  const Config& config,
  const std::shared_ptr<mw::Middleware>& mw
)
    : Node(node_name, mw)
    , config_(config)
    , pub_(create_publisher<sensor_msgs::Imu>(config.topic_name))
    , pimpl_(std::make_unique<Impl>()) {
}

ImuNode::~ImuNode() = default;

void ImuNode::spin_once() {
  pimpl_->spin_once(pub_);
  std::this_thread::sleep_for(std::chrono::milliseconds(config_.delay_ms));
}

bool ImuNode::valid() const noexcept {
  return Node::valid() && pimpl_->valid();
}

} // namespace jr

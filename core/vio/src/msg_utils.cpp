#include "vio/msg_utils.hpp"

#include <cmath>

namespace jr::vio {

int deserialize_encoding(const std::string& encoding) {
  if (encoding == "8UC1")
    return CV_8UC1;
  if (encoding == "8UC3")
    return CV_8UC3;
  return -1;
}

std::string serialize_encoding(int cv_type) {
  switch (cv_type) {
  case CV_8UC1:
    return "8UC1";
  case CV_8UC3:
    return "8UC3";
  default:
    return "unknown";
  }
}

cv::Mat from_image_msg(const sensor_msgs::Image& msg) {
  return cv::Mat(
           msg.height(),
           msg.width(),
           deserialize_encoding(msg.encoding()),
           const_cast<char*>(msg.data().data()),
           msg.step()
  )
    .clone();
}

double get_timestamp_sec(const sensor_msgs::Image& msg) {
  return static_cast<double>(msg.header().stamp().sec()) +
         static_cast<double>(msg.header().stamp().nsec()) * 1e-9;
}

double get_timestamp_sec(const sensor_msgs::Imu& msg) {
  return static_cast<double>(msg.header().stamp().sec()) +
         static_cast<double>(msg.header().stamp().nsec()) * 1e-9;
}

cv::Mat quaternion_to_rotation_matrix(
  double qx,
  double qy,
  double qz,
  double qw
) {
  // Normalize quaternion
  double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > 1e-10) {
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
  }

  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  R.at<double>(0, 0) = 1 - 2 * (qy * qy + qz * qz);
  R.at<double>(0, 1) = 2 * (qx * qy - qz * qw);
  R.at<double>(0, 2) = 2 * (qx * qz + qy * qw);
  R.at<double>(1, 0) = 2 * (qx * qy + qz * qw);
  R.at<double>(1, 1) = 1 - 2 * (qx * qx + qz * qz);
  R.at<double>(1, 2) = 2 * (qy * qz - qx * qw);
  R.at<double>(2, 0) = 2 * (qx * qz - qy * qw);
  R.at<double>(2, 1) = 2 * (qy * qz + qx * qw);
  R.at<double>(2, 2) = 1 - 2 * (qx * qx + qy * qy);

  return R;
}

std::tuple<double, double, double, double> rotation_matrix_to_quaternion(
  const cv::Mat& R
) {
  double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
  double qw, qx, qy, qz;

  if (trace > 0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
    qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
    qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
  } else if (R.at<double>(0, 0) > R.at<double>(1, 1) &&
             R.at<double>(0, 0) > R.at<double>(2, 2)) {
    double s =
      2.0 * std::sqrt(
              1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2)
            );
    qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
    qx = 0.25 * s;
    qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
    qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
  } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
    double s =
      2.0 * std::sqrt(
              1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2)
            );
    qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
    qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
    qy = 0.25 * s;
    qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
  } else {
    double s =
      2.0 * std::sqrt(
              1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1)
            );
    qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
    qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
    qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
    qz = 0.25 * s;
  }

  return {qx, qy, qz, qw};
}

geometry_msgs::PoseStamped create_pose_msg(
  const cv::Mat& R,
  const cv::Mat& t,
  double timestamp_sec
) {
  geometry_msgs::PoseStamped msg;

  auto* header = msg.mutable_header();
  header->set_frame_id("world");
  header->mutable_stamp()->set_sec(static_cast<int32_t>(timestamp_sec));
  header->mutable_stamp()->set_nsec(
    static_cast<int32_t>((timestamp_sec - std::floor(timestamp_sec)) * 1e9)
  );

  // Position
  msg.mutable_pose()->mutable_position()->set_x(t.at<double>(0));
  msg.mutable_pose()->mutable_position()->set_y(t.at<double>(1));
  msg.mutable_pose()->mutable_position()->set_z(t.at<double>(2));

  // Orientation
  auto [qx, qy, qz, qw] = rotation_matrix_to_quaternion(R);
  msg.mutable_pose()->mutable_orientation()->set_x(qx);
  msg.mutable_pose()->mutable_orientation()->set_y(qy);
  msg.mutable_pose()->mutable_orientation()->set_z(qz);
  msg.mutable_pose()->mutable_orientation()->set_w(qw);

  return msg;
}

} // namespace jr::vio

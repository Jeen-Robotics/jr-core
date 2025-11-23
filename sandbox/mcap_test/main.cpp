#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <sensor_msgs.pb.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <iostream>
#include <random>
#include <queue>
#include <unordered_set>

#define NS_PER_MS 1000000
#define NS_PER_S 1000000000
#define POINTS_PER_CLOUD 1000
#define FIELDS_PER_POINT 3

namespace {
std::string serialize_encoding(const int encoding) {
  switch (encoding) {
  case CV_8UC1:
    return "8UC1";
  case CV_8UC3:
    return "8UC3";
  default:
    return "unknown";
  }
}

sensor_msgs::Image to_msg(const cv::Mat& frame) {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()
                      )
                        .count();
  const auto now_sec = static_cast<double>(now_ns) * 1e-9;

  sensor_msgs::Image msg;

  auto* header = msg.mutable_header();
  header->mutable_stamp()->set_sec(static_cast<int32_t>(now_sec));
  header->mutable_stamp()->set_nsec(static_cast<int32_t>(now_ns % NS_PER_S));
  header->set_frame_id("camera");
  header->set_seq(0);

  msg.set_encoding(serialize_encoding(frame.type()));
  msg.set_is_bigendian(false);
  msg.set_width(frame.cols);
  msg.set_height(frame.rows);
  msg.set_step(frame.step);
  msg.set_data(frame.data, frame.total() * frame.elemSize());

  return msg;
}
} // namespace

// PointGenerator generates random points on a sphere.
class PointGenerator {
  std::mt19937 _generator;
  std::uniform_real_distribution<float> _distribution;

public:
  explicit PointGenerator(const uint32_t seed = 0)
      : _generator(seed)
      , _distribution(0.0, 1.0) {
  }

  // next produces a random point on the unit sphere, scaled by `scale`.
  std::tuple<float, float, float> next(const float scale) {
    const float theta =
      2 * static_cast<float>(M_PI) * _distribution(_generator);
    const float phi = acos(1.f - (2.f * _distribution(_generator)));
    float x = sin(phi) * cos(theta) * scale;
    float y = (sin(phi) * sin(theta)) * scale;
    float z = cos(phi) * scale;
    return {x, y, z};
  }
};

google::protobuf::FileDescriptorSet build_file_descriptor_set(
  const google::protobuf::Descriptor* toplevelDescriptor
) {
  google::protobuf::FileDescriptorSet fdSet;
  std::queue<const google::protobuf::FileDescriptor*> toAdd;
  toAdd.push(toplevelDescriptor->file());
  std::unordered_set<std::string> seenDependencies;
  while (!toAdd.empty()) {
    const google::protobuf::FileDescriptor* next = toAdd.front();
    toAdd.pop();
    next->CopyTo(fdSet.add_file());
    for (int i = 0; i < next->dependency_count(); ++i) {
      const auto& dep = next->dependency(i);
      if (seenDependencies.find(dep->name()) == seenDependencies.end()) {
        seenDependencies.insert(dep->name());
        toAdd.push(dep);
      }
    }
  }
  return fdSet;
}

int main() {
  const std::string output_filename = "test.mcap";

  mcap::McapWriter writer;

  const auto options = mcap::McapWriterOptions("");
  auto res = writer.open(output_filename, options);
  if (!res.ok()) {
    std::cerr << "Error opening output file " << output_filename << std::endl;
    return 1;
  }

  const auto* descriptor = sensor_msgs::PointCloud2::descriptor();
  mcap::Schema schema(
    descriptor->full_name(),
    "protobuf",
    build_file_descriptor_set(descriptor).SerializeAsString()
  );

  writer.addSchema(schema);

  mcap::Channel channel("/pointcloud", "protobuf", schema.id);

  writer.addChannel(channel);

  // Add schema and channel for Image messages
  const auto* image_descriptor = sensor_msgs::Image::descriptor();
  mcap::Schema image_schema(
    image_descriptor->full_name(),
    "protobuf",
    build_file_descriptor_set(image_descriptor).SerializeAsString()
  );

  writer.addSchema(image_schema);

  mcap::Channel image_channel("/camera/image", "protobuf", image_schema.id);

  writer.addChannel(image_channel);

  sensor_msgs::PointCloud2 pcl;
  pcl.set_point_step(sizeof(float) * FIELDS_PER_POINT);

  const char* const field_names[] = {"x", "y", "z"};
  int field_offset = 0;
  for (const auto& name : field_names) {
    auto field = pcl.add_fields();
    field->set_name(name);
    field->set_offset(field_offset);
    field->set_datatype(google::protobuf::FieldDescriptorProto_Type_TYPE_FLOAT);
    field_offset += sizeof(float);
  }

  pcl.mutable_data()->append(
    POINTS_PER_CLOUD * FIELDS_PER_POINT * sizeof(float),
    '\0'
  );

  mcap::Timestamp start_time =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()
    )
      .count();

  PointGenerator pointGenerator;
  for (size_t i = 0; i < 100; ++i) {
    mcap::Timestamp cloud_time =
      start_time + (static_cast<uint64_t>(i) * 100 * NS_PER_MS);

    // write 1000 points into each pointcloud message.
    size_t offset = 0;
    for (int point_index = 0; point_index < POINTS_PER_CLOUD; ++point_index) {
      auto [x, y, z] = pointGenerator.next(1.0);
      char* data = pcl.mutable_data()->data();
      std::memcpy(&data[offset], reinterpret_cast<const char*>(&x), sizeof(x));
      offset += sizeof(x);
      std::memcpy(&data[offset], reinterpret_cast<const char*>(&y), sizeof(y));
      offset += sizeof(y);
      std::memcpy(&data[offset], reinterpret_cast<const char*>(&z), sizeof(z));
      offset += sizeof(z);
    }

    auto serialized = pcl.SerializeAsString();

    mcap::Message msg;
    msg.channelId = channel.id;
    msg.publishTime = cloud_time;
    msg.logTime = cloud_time;
    msg.data = reinterpret_cast<const std::byte*>(serialized.data());
    msg.dataSize = serialized.size();

    res = writer.write(msg);
    if (!res.ok()) {
      std::cerr << "Error writing to output file " << output_filename
                << std::endl;
      writer.terminate();
      return 1;
    }
  }

  // Open video capture from camera index 0
  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open camera with index 0" << std::endl;
    writer.close();
    return 1;
  }

  // Read and write images to MCAP
  mcap::Timestamp image_start_time =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()
    )
      .count();

  cv::Mat frame;
  size_t frame_count = 0;
  constexpr size_t max_frames = 100; // Limit to 100 frames for testing

  while (frame_count < max_frames) {
    cap >> frame;
    if (frame.empty()) {
      std::cerr << "Warning: Empty frame captured" << std::endl;
      break;
    }

    // Convert OpenCV Mat to sensor_msgs::Image
    sensor_msgs::Image image_msg = to_msg(frame);

    // Calculate timestamp
    mcap::Timestamp image_time =
      image_start_time + (static_cast<uint64_t>(frame_count) * 100 * NS_PER_MS);

    // Update timestamp in message header
    auto* header = image_msg.mutable_header();
    const auto msg_sec = static_cast<double>(image_time) * 1e-9;
    header->mutable_stamp()->set_sec(static_cast<int32_t>(msg_sec));
    header->mutable_stamp()->set_nsec(static_cast<int32_t>(image_time % NS_PER_S));
    header->set_seq(static_cast<uint32_t>(frame_count));

    // Serialize and write to MCAP
    auto serialized = image_msg.SerializeAsString();

    mcap::Message msg;
    msg.channelId = image_channel.id;
    msg.publishTime = image_time;
    msg.logTime = image_time;
    msg.data = reinterpret_cast<const std::byte*>(serialized.data());
    msg.dataSize = serialized.size();

    res = writer.write(msg);
    if (!res.ok()) {
      std::cerr << "Error writing image to output file " << output_filename
                << std::endl;
      writer.terminate();
      return 1;
    }

    frame_count++;
  }

  cap.release();

  writer.close();

  return 0;
}
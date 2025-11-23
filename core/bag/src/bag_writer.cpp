#include "bag/bag_writer.hpp"

#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <middleware/middleware.hpp>
#include <sensor_msgs.pb.h>

#include <chrono>
#include <filesystem>
#include <queue>
#include <unordered_set>

#include "bag.pb.h"
#include "video_encoder.hpp"

namespace {

std::string sanitize_topic_name(const std::string& topic) {
  std::string result = topic;
  // Replace slashes and special characters with underscores
  for (char& c : result) {
    if (c == '/' || c == ' ' || c == ':') {
      c = '_';
    }
  }
  // Remove leading underscores
  while (!result.empty() && result[0] == '_') {
    result = result.substr(1);
  }
  return result;
}

std::uint64_t now_ns() {
  return static_cast<std::uint64_t>(
    std::chrono::time_point_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now()
    )
      .time_since_epoch()
      .count()
  );
}

std::string serialize(const google::protobuf::Message& msg) {
  std::string res;
  msg.SerializeToString(&res);
  return res;
}

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

} // namespace

namespace jr::mw {

class BagWriter::Impl {
public:
  mcap::McapWriter writer;
  std::map<std::string, mcap::SchemaId> schema_ids;
  std::map<std::string, mcap::ChannelId> channel_ids;
};

BagWriter::BagWriter(const std::string& path)
    : impl_(std::make_unique<Impl>())
    , path_(path)
    , mw_(get()) {
}

BagWriter::BagWriter(const std::string& path, std::shared_ptr<Middleware> mw)
    : impl_(std::make_unique<Impl>())
    , path_(path)
    , mw_(std::move(mw)) {
}

BagWriter::~BagWriter() {
  close();
}

bool BagWriter::open() {
  std::lock_guard lock(out_mutex_);

  if (is_open_) {
    return true;
  }

  mcap::McapWriterOptions options("");
  auto res = impl_->writer.open(path_, options);
  is_open_ = res.ok();

  return is_open_;
}

void BagWriter::close() {
  // Close all video encoders (flushes data to video files)
  video_encoders_.clear();

  std::lock_guard lock(out_mutex_);

  active_subs_.clear();
  if (is_open_) {
    impl_->writer.close();
    is_open_ = false;
  }
}

void BagWriter::set_video_config(VideoEncoderConfig config) {
  video_config_ = std::move(config);
}

void BagWriter::record_topic(const std::string& topic) {
  if (!mw_) {
    return;
  }

  open();

  auto sub = mw_->subscribe_any(
    topic,
    [this, topic](
      const std::string& type_name,
      const google::protobuf::Message& msg
    ) {
      const auto ts_ns = now_ns();

      // Process image messages
      if (const auto* image_msg =
            dynamic_cast<const sensor_msgs::Image*>(&msg)) {
        // Get or create video encoder for this topic
        auto& encoder = video_encoders_[topic];
        if (!encoder) {
          std::string video_path = get_video_path_for_topic(topic);
          encoder =
            std::make_unique<VideoEncoder>(topic, video_path, video_config_);
        }

        // Add frame to encoder and write reference immediately
        if (auto frame_ref = encoder->add_frame(*image_msg)) {
          // Create VideoFrameReference protobuf message
          VideoFrameReference ref_msg;
          ref_msg.set_topic(frame_ref->topic);
          ref_msg.set_video_file(frame_ref->video_file);
          ref_msg.set_frame_index(frame_ref->frame_index);
          ref_msg.set_width(frame_ref->width);
          ref_msg.set_height(frame_ref->height);
          ref_msg.set_encoding(frame_ref->encoding);
          ref_msg.set_seq(frame_ref->seq);

          // Write frame reference to bag
          write_record(topic, ref_msg, ts_ns);
        }
        return;
      }

      // Regular message - serialize and write
      write_record(topic, msg, ts_ns);
    }
  );

  active_subs_.emplace_back(std::move(sub));
}

void BagWriter::record_all() {
  if (!mw_) {
    return;
  }

  open();

  for (const auto& ti : mw_->get_topic_names_and_types()) {
    record_topic(ti.name);
  }
}

void BagWriter::stop() {
  for (auto& sub : active_subs_) {
    sub.unsubscribe();
  }
}

void BagWriter::write_record(
  const std::string& topic,
  const google::protobuf::Message& msg,
  std::uint64_t ts_ns
) {
  std::lock_guard lock(out_mutex_);

  if (!is_open_) {
    return;
  }

  // Check if we have a schema for this type
  const auto* descriptor = msg.GetDescriptor();
  std::string type_full_name = descriptor->full_name();

  if (impl_->schema_ids.find(type_full_name) == impl_->schema_ids.end()) {
    // Register schema
    mcap::Schema schema(
      type_full_name,
      "protobuf",
      build_file_descriptor_set(descriptor).SerializeAsString()
    );
    impl_->writer.addSchema(schema);
    impl_->schema_ids[type_full_name] = schema.id;
  }

  // Check if we have a channel for this topic
  if (impl_->channel_ids.find(topic) == impl_->channel_ids.end()) {
    // Register channel
    mcap::Channel channel(topic, "protobuf", impl_->schema_ids[type_full_name]);
    impl_->writer.addChannel(channel);
    impl_->channel_ids[topic] = channel.id;
  }

  // Write message
  std::string serialized;
  msg.SerializeToString(&serialized);

  mcap::Message mcap_msg;
  mcap_msg.channelId = impl_->channel_ids[topic];
  mcap_msg.sequence = 0; // Optional: keep track of sequence if needed
  mcap_msg.logTime = ts_ns;
  mcap_msg.publishTime = ts_ns;
  mcap_msg.data = reinterpret_cast<const std::byte*>(serialized.data());
  mcap_msg.dataSize = serialized.size();

  const auto res = impl_->writer.write(mcap_msg);
  if (!res.ok()) {
    std::cerr << "Error writing message to bag: " << res.message << '\n';
  }
}

std::string BagWriter::get_video_path_for_topic(const std::string& topic
) const {
  // Extract bag directory and base name
  const std::filesystem::path bag_path(path_);
  const auto bag_dir = bag_path.parent_path();
  const auto bag_base = bag_path.stem().string(); // filename without extension

  // Create video filename: bag-name_topic.avi
  const auto sanitized_topic = sanitize_topic_name(topic);
  const auto video_filename = bag_base + "_" + sanitized_topic + ".avi";

  // Full path to a video file
  const auto video_path = bag_dir / video_filename;
  return video_path.string();
}

} // namespace jr::mw

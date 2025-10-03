#include "bag/bag_writer.hpp"

#include "video_encoder.hpp"

#include <bag.pb.h>
#include <middleware/middleware.hpp>
#include <sensor_msgs.pb.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <zstd.h>

namespace jr::mw {

BagWriter::BagWriter(const std::string& path)
    : path_(path)
    , mw_(get()) {
}

BagWriter::BagWriter(const std::string& path, std::shared_ptr<Middleware> mw)
    : path_(path)
    , mw_(std::move(mw)) {
}

BagWriter::~BagWriter() {
  close();
}

void BagWriter::set_video_config(VideoEncoderConfig config) {
  video_config_ = std::move(config);
}

bool BagWriter::open() {
  std::lock_guard<std::mutex> lock(out_mutex_);

  if (out_.is_open()) {
    return true;
  }

  out_.open(path_, std::ios::binary | std::ios::out | std::ios::trunc);

  if (out_.is_open()) {
    // Write protobuf header
    BagHeader header;
    header.set_version(2); // Version 2 with compression support
    header.set_format_id("JR_BAG");
    header.set_compression(COMPRESSION_ZSTD);

    std::string header_data;
    header.SerializeToString(&header_data);

    // Write header size followed by header data
    uint32_t header_size = static_cast<uint32_t>(header_data.size());
    out_.write(
      reinterpret_cast<const char*>(&header_size),
      sizeof(header_size)
    );
    out_.write(header_data.data(), header_data.size());
    out_.flush();
  }

  return out_.is_open();
}

void BagWriter::close() {
  // Close all video encoders (flushes data to video files)
  video_encoders_.clear();

  std::lock_guard<std::mutex> lock(out_mutex_);

  active_subs_.clear();
  if (out_.is_open()) {
    out_.close();
  }
}

std::string BagWriter::sanitize_topic_name(const std::string& topic) {
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

std::string BagWriter::get_video_path_for_topic(const std::string& topic) {
  // Extract bag directory and base name
  std::filesystem::path bag_path(path_);
  std::filesystem::path bag_dir = bag_path.parent_path();
  std::string bag_base = bag_path.stem().string(); // filename without extension

  // Create video filename: bagname_topic.avi
  std::string sanitized_topic = sanitize_topic_name(topic);
  std::string video_filename = bag_base + "_" + sanitized_topic + ".avi";

  // Full path to video file
  std::filesystem::path video_path = bag_dir / video_filename;
  return video_path.string();
}

void BagWriter::write_record(
  const std::string& topic,
  const std::string& type_full_name,
  const std::string& payload,
  std::uint64_t ts_ns
) {
  std::lock_guard<std::mutex> lock(out_mutex_);

  if (!out_.is_open()) {
    return;
  }

  // Create protobuf record
  BagRecord record;
  record.set_topic(topic);
  record.set_type_full_name(type_full_name);
  record.set_payload(payload);
  record.set_timestamp_ns(ts_ns);

  // Serialize record
  std::string record_data;
  record.SerializeToString(&record_data);

  // Compress record with zstd
  size_t const compressed_bound = ZSTD_compressBound(record_data.size());
  std::string compressed_data(compressed_bound, '\0');

  size_t const compressed_size = ZSTD_compress(
    compressed_data.data(),
    compressed_bound,
    record_data.data(),
    record_data.size(),
    3 // Compression level: 3 is a good balance of speed and ratio
  );

  if (ZSTD_isError(compressed_size)) {
    // If compression fails, write uncompressed (shouldn't happen)
    uint32_t record_size = static_cast<uint32_t>(record_data.size());
    out_.write(
      reinterpret_cast<const char*>(&record_size),
      sizeof(record_size)
    );
    out_.write(record_data.data(), record_data.size());
    return;
  }

  compressed_data.resize(compressed_size);

  // Write compressed record size followed by compressed data
  uint32_t record_size = static_cast<uint32_t>(compressed_data.size());
  out_.write(reinterpret_cast<const char*>(&record_size), sizeof(record_size));
  out_.write(compressed_data.data(), compressed_data.size());
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
      auto now = std::chrono::time_point_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now()
      )
                   .time_since_epoch()
                   .count();
      std::uint64_t ts_ns = static_cast<std::uint64_t>(now);

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
        auto frame_ref = encoder->add_frame(*image_msg);
        if (frame_ref) {
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
          std::string payload;
          ref_msg.SerializeToString(&payload);
          write_record(topic, "jr.mw.VideoFrameReference", payload, ts_ns);
        }
        return;
      }

      // Regular message - serialize and write
      std::string payload;
      msg.SerializeToString(&payload);
      write_record(topic, type_name, payload, ts_ns);
    }
  );

  active_subs_.push_back(std::move(sub));
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

} // namespace jr::mw

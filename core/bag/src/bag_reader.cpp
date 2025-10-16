#include "bag/bag_reader.hpp"

#include <zstd.h>

#include <bag.pb.h>
#include <middleware/middleware.hpp>
#include <sensor_msgs.pb.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include "video_decoder.hpp"

namespace {

constexpr size_t MAX_RECORD_SIZE = 100 * 1024 * 1024; // 100MB max

}

namespace jr::mw {

BagReader::BagReader(const std::string& path)
    : path_(path)
    , mw_(get())
    , video_decoder_(std::make_unique<VideoDecoder>()) {
}

BagReader::BagReader(const std::string& path, std::shared_ptr<Middleware> mw)
    : path_(path)
    , mw_(std::move(mw))
    , video_decoder_(std::make_unique<VideoDecoder>()) {
}

BagReader::~BagReader() {
  close();
}

bool BagReader::open() {
  if (in_.is_open())
    return true;
  in_.open(path_, std::ios::binary | std::ios::in);

  if (in_.is_open()) {
    // Read and validate protobuf header
    uint32_t header_size = 0;
    if (!in_.read(reinterpret_cast<char*>(&header_size), sizeof(header_size))) {
      std::cerr << "BagReader::open: failed to read header size\n";
      in_.close();
      return false;
    }

    std::string header_data;
    header_data.resize(header_size);
    if (!in_.read(header_data.data(), header_size)) {
      std::cerr << "BagReader::open: failed to read header data\n";
      in_.close();
      return false;
    }

    BagHeader header;
    if (!header.ParseFromString(header_data)) {
      std::cerr << "BagReader::open: failed to parse header\n";
      in_.close();
      return false;
    }

    if (header.format_id() != "JR_BAG") {
      std::cerr
        << "BagReader::open: invalid bag file format - expected JR_BAG, got "
        << header.format_id() << "\n";
      in_.close();
      return false;
    }

    // Store compression type for reading records
    compression_type_ = static_cast<int>(header.compression());

    std::cout << "Opened bag file version " << header.version();
    if (compression_type_ == 1) {
      std::cout << " (zstd compressed)";
    }
    std::cout << "\n";
  }

  return in_.is_open();
}

void BagReader::close() {
  if (in_.is_open())
    in_.close();
}

bool BagReader::read_record(
  std::string& topic,
  std::string& type,
  std::string& payload,
  std::uint64_t& ts
) {
  if (!in_.is_open()) {
    std::cerr << "BagReader::read_record: not open\n";
    return false;
  }

  if (in_.eof()) {
    return false;
  }

  // Read record size
  uint32_t record_size = 0;
  if (!in_.read(reinterpret_cast<char*>(&record_size), sizeof(record_size))) {
    // EOF is expected, not an error
    if (in_.eof()) {
      return false;
    }
    std::cerr << "BagReader::read_record: failed to read record size\n";
    return false;
  }

  // Validate record size to detect corruption
  if (record_size > MAX_RECORD_SIZE) {
    std::cerr << "BagReader::read_record: invalid record size " << record_size
              << " - possible corruption\n";
    return false;
  }

  // Read record data
  std::string compressed_data;
  compressed_data.resize(record_size);
  if (!in_.read(compressed_data.data(), record_size)) {
    std::cerr << "BagReader::read_record: failed to read record data\n";
    return false;
  }

  std::string record_data;

  // Decompress if needed
  if (compression_type_ == 1) { // COMPRESSION_ZSTD
    // Get decompressed size
    unsigned long long const decompressed_size =
      ZSTD_getFrameContentSize(compressed_data.data(), compressed_data.size());

    if (decompressed_size == ZSTD_CONTENTSIZE_ERROR ||
        decompressed_size == ZSTD_CONTENTSIZE_UNKNOWN) {
      std::cerr << "BagReader::read_record: invalid zstd frame\n";
      return false;
    }

    record_data.resize(decompressed_size);
    size_t const actual_size = ZSTD_decompress(
      record_data.data(),
      decompressed_size,
      compressed_data.data(),
      compressed_data.size()
    );

    if (ZSTD_isError(actual_size)) {
      std::cerr << "BagReader::read_record: zstd decompression failed: "
                << ZSTD_getErrorName(actual_size) << "\n";
      return false;
    }
  } else {
    // No compression
    record_data = std::move(compressed_data);
  }

  // Parse protobuf record
  BagRecord record;
  if (!record.ParseFromString(record_data)) {
    std::cerr << "BagReader::read_record: failed to parse record\n";
    return false;
  }

  topic = record.topic();
  type = record.type_full_name();
  payload = record.payload();
  ts = record.timestamp_ns();

  return true;
}

std::string BagReader::resolve_video_path(
  const std::string& relative_path
) const {
  // Get bag directory
  const std::filesystem::path bag_path(path_);
  const auto bag_dir = bag_path.parent_path();

  // Resolve a relative video path
  const auto video_path = bag_dir / relative_path;
  return video_path.string();
}

void BagReader::play(double rate) {
  if (!mw_) {
    return;
  }

  if (!open()) {
    return;
  }

  std::string topic, type, payload;
  std::uint64_t ts = 0;
  std::uint64_t first_ts = 0;
  auto wall_start = std::chrono::steady_clock::now();
  is_playing_.store(true);

  while (is_playing_.load() && read_record(topic, type, payload, ts)) {
    if (first_ts == 0) {
      first_ts = ts;
    }

    // Check if this is a video frame reference
    if (type == "jr.mw.VideoFrameReference") {
      // Deserialize frame reference
      VideoFrameReference frame_ref;
      if (!frame_ref.ParseFromString(payload)) {
        std::cerr << "BagReader: failed to parse video frame reference\n";
        continue;
      }

      // Resolve a video file path
      std::string video_path = resolve_video_path(frame_ref.video_file());

      // Decode the frame
      sensor_msgs::Image frame_msg;
      if (!video_decoder_->decode_frame(
            video_path,
            frame_ref.frame_index(),
            frame_ref.width(),
            frame_ref.height(),
            frame_ref.encoding(),
            frame_ref.seq(),
            frame_msg
          )) {
        std::cerr << "BagReader: failed to decode frame "
                  << frame_ref.frame_index() << " from " << video_path << "\n";
        continue;
      }

      // Publish as sensor_msgs::Image
      std::string frame_payload;
      frame_msg.SerializeToString(&frame_payload);

      // Apply playback rate timing
      if (rate > 0) {
        std::uint64_t target_ns =
          static_cast<std::uint64_t>((ts - first_ts) / rate);
        const auto target = wall_start + std::chrono::nanoseconds(target_ns);
        const auto now = std::chrono::steady_clock::now();
        if (target > now) {
          std::this_thread::sleep_for(target - now);
        }
      }

      mw_->publish_serialized(
        frame_ref.topic(),
        "sensor_msgs.Image",
        frame_payload
      );
      continue;
    }

    // Regular message
    if (rate > 0) {
      std::uint64_t target_ns =
        static_cast<std::uint64_t>((ts - first_ts) / rate);
      auto target = wall_start + std::chrono::nanoseconds(target_ns);
      auto now = std::chrono::steady_clock::now();
      if (target > now) {
        std::this_thread::sleep_for(target - now);
      }
    }

    mw_->publish_serialized(topic, type, payload);
  }
}

void BagReader::stop() {
  is_playing_.store(false);
}

} // namespace jr::mw

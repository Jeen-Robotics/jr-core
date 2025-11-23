#include "bag/bag_reader.hpp"

#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include <bag.pb.h>
#include <middleware/middleware.hpp>
#include <sensor_msgs.pb.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include "video_decoder.hpp"

namespace {

// Helper to look up schema by ID from the reader
std::optional<mcap::Schema> get_schema(
  const mcap::McapReader& reader,
  mcap::SchemaId id
) {
  const auto& schemas = reader.schemas();
  auto it = schemas.find(id);
  if (it != schemas.end()) {
    return *it->second;
  }
  return std::nullopt;
}

std::optional<mcap::Channel> get_channel(
  const mcap::McapReader& reader,
  mcap::ChannelId id
) {
  const auto& channels = reader.channels();
  auto it = channels.find(id);
  if (it != channels.end()) {
    return *it->second;
  }
  return std::nullopt;
}

} // namespace

namespace jr::mw {

class BagReader::Impl {
public:
  mcap::McapReader reader;
};

BagReader::BagReader(const std::string& path)
    : impl_(std::make_unique<Impl>())
    , path_(path)
    , mw_(get())
    , video_decoder_(std::make_unique<VideoDecoder>()) {
}

BagReader::BagReader(const std::string& path, std::shared_ptr<Middleware> mw)
    : impl_(std::make_unique<Impl>())
    , path_(path)
    , mw_(std::move(mw))
    , video_decoder_(std::make_unique<VideoDecoder>()) {
}

BagReader::~BagReader() {
  close();
}

bool BagReader::open() {
  // Check if already open - difficult with McapReader as it doesn't expose
  // is_open easily without try/catch or storing state But we can try opening.

  auto res = impl_->reader.open(path_);
  if (!res.ok()) {
    std::cerr << "Failed to open MCAP file: " << res.message << "\n";
    return false;
  }

  return true;
}

void BagReader::close() {
  impl_->reader.close();
}

std::string BagReader::resolve_video_path(const std::string& relative_path
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

  std::uint64_t first_ts = 0;
  auto wall_start = std::chrono::steady_clock::now();
  is_playing_.store(true);

  // Helper cache for channel/schema info
  struct ChannelInfo {
    std::string topic;
    std::string type;
  };
  std::map<mcap::ChannelId, ChannelInfo> channel_cache;

  // Iterate over messages
  for (const auto& msgView : impl_->reader.readMessages()) {
    if (!is_playing_.load())
      break;

    const auto& msg = msgView.message;

    // Resolve topic and type if not cached
    if (channel_cache.find(msg.channelId) == channel_cache.end()) {
      auto channelOpt = get_channel(impl_->reader, msg.channelId);
      if (channelOpt) {
        std::string typeName = "unknown";
        if (channelOpt->schemaId != 0) {
          auto schemaOpt = get_schema(impl_->reader, channelOpt->schemaId);
          if (schemaOpt) {
            typeName = schemaOpt->name;
          }
        }
        channel_cache[msg.channelId] = {channelOpt->topic, typeName};
      } else {
        // Skip unknown channels
        continue;
      }
    }

    const auto& info = channel_cache[msg.channelId];
    const std::string& topic = info.topic;
    const std::string& type = info.type;
    std::string payload(reinterpret_cast<const char*>(msg.data), msg.dataSize);
    std::uint64_t ts = msg.logTime; // Use logTime as timestamp

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
        if (ts > first_ts) {
          std::uint64_t target_ns =
            static_cast<std::uint64_t>((ts - first_ts) / rate);
          const auto target = wall_start + std::chrono::nanoseconds(target_ns);
          const auto now = std::chrono::steady_clock::now();
          if (target > now) {
            std::this_thread::sleep_for(target - now);
          }
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
      if (ts > first_ts) {
        std::uint64_t target_ns =
          static_cast<std::uint64_t>((ts - first_ts) / rate);
        auto target = wall_start + std::chrono::nanoseconds(target_ns);
        auto now = std::chrono::steady_clock::now();
        if (target > now) {
          std::this_thread::sleep_for(target - now);
        }
      }
    }

    mw_->publish_serialized(topic, type, payload);
  }
}

void BagReader::stop() {
  is_playing_.store(false);
}

} // namespace jr::mw

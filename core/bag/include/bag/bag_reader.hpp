#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

namespace jr::mw {

class Middleware;
class VideoDecoder;

class BagReader {
public:
  explicit BagReader(const std::string& path);
  explicit BagReader(const std::string& path, std::shared_ptr<Middleware> mw);
  ~BagReader();

  bool open();
  void close();

  // Play back into middleware; if rate <= 0, play as fast as possible
  void play(double rate = 1.0);
  void stop();

private:
  std::string path_;
  std::shared_ptr<Middleware> mw_;
  std::ifstream in_;
  std::atomic_bool is_playing_ = false;
  int compression_type_ = 0; // 0 = COMPRESSION_NONE, 1 = COMPRESSION_ZSTD

  // Video decoder (shared across all video files)
  std::unique_ptr<VideoDecoder> video_decoder_;

  bool read_record(
    std::string& topic,
    std::string& type,
    std::string& payload,
    std::uint64_t& ts
  );

  // Resolve a relative video file path to an absolute path
  std::string resolve_video_path(const std::string& relative_path) const;
};

} // namespace jr::mw

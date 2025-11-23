#pragma once

#include <atomic>
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
  class Impl;
  std::unique_ptr<Impl> impl_;

  std::string path_;
  std::shared_ptr<Middleware> mw_;
  std::atomic_bool is_playing_ = false;

  // Video decoder (shared across all video files)
  std::unique_ptr<VideoDecoder> video_decoder_;

  // Resolve a relative video file path to an absolute path
  std::string resolve_video_path(const std::string& relative_path) const;
};

} // namespace jr::mw

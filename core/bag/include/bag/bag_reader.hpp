#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

namespace jr::mw {

class Middleware;

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

  bool read_record(
    std::string& topic,
    std::string& type,
    std::string& payload,
    std::uint64_t& ts
  );
};

} // namespace jr::mw

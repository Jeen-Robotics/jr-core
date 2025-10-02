#include "bag/bag_reader.hpp"

#include <bag.pb.h>
#include <middleware/middleware.hpp>

#include <chrono>
#include <iostream>
#include <thread>

namespace jr::mw {

BagReader::BagReader(const std::string& path)
    : path_(path)
    , mw_(get()) {
}

BagReader::BagReader(const std::string& path, std::shared_ptr<Middleware> mw)
    : path_(path)
    , mw_(std::move(mw)) {
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

    std::cout << "Opened bag file version " << header.version() << "\n";
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
  const size_t MAX_RECORD_SIZE = 100 * 1024 * 1024; // 100MB max
  if (record_size > MAX_RECORD_SIZE) {
    std::cerr << "BagReader::read_record: invalid record size " << record_size
              << " - possible corruption\n";
    return false;
  }

  // Read record data
  std::string record_data;
  record_data.resize(record_size);
  if (!in_.read(record_data.data(), record_size)) {
    std::cerr << "BagReader::read_record: failed to read record data\n";
    return false;
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

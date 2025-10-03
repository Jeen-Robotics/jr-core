#include "bag/bag_writer.hpp"

#include <bag.pb.h>
#include <middleware/middleware.hpp>

#include <chrono>
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
  std::lock_guard<std::mutex> lock(out_mutex_);

  active_subs_.clear();
  if (out_.is_open()) {
    out_.close();
  }
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
      std::string payload;
      msg.SerializeToString(&payload);
      auto now = std::chrono::time_point_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now()
      )
                   .time_since_epoch()
                   .count();
      write_record(topic, type_name, payload, static_cast<std::uint64_t>(now));
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

#include "jr_api/runner_api.h"

#include <bag/bag_writer.hpp>
#include <camera_node/camera_node.hpp>
#include <middleware/middleware.hpp>

#include <chrono>
#include <filesystem>
#include <memory>
#include <thread>

namespace {

constexpr int MAX_NUM_CAMERAS = 3;

struct Runner {
  std::thread spin_thread;
  std::unique_ptr<jr::mw::BagWriter> writer;
};

Runner& get() {
  static Runner runner{};
  return runner;
}

std::string get_time_string() {
  const auto now = std::chrono::system_clock::now();
  const auto now_c = std::chrono::system_clock::to_time_t(now);
  const auto time = *std::localtime(&now_c);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%d%m%Y_%H%M%S", &time);
  return std::string(buf);
}

} // namespace

void init() {
  jr::mw::init();

  get().spin_thread = std::thread([]() {
    std::vector<std::shared_ptr<jr::mw::Node>> nodes;

    for (int i = 0; i < MAX_NUM_CAMERAS; ++i) {
      const auto node_name = "camera" + std::to_string(i);
      const auto topic_name = "/" + node_name + "/frame";
      const auto pub_node = std::make_shared<jr::CameraNode>(
        node_name,
        jr::CameraNode::Config{topic_name, i}
      );
      if (pub_node->valid()) {
        nodes.emplace_back(pub_node);
      }
    }

    jr::mw::spin(nodes);
  });
}

void start_recording(const char* save_directory) {
  const auto bag_name = "jr_" + get_time_string();
  const auto bag_path = std::filesystem::path(save_directory) / bag_name;
  std::filesystem::create_directory(bag_path);

  const auto filename = bag_name + ".bag";
  const auto file_path = bag_path / filename;
  get().writer = std::make_unique<jr::mw::BagWriter>(file_path.string());
  get().writer->set_video_config(
    jr::mw::VideoEncoderConfig{
      .codec = jr::mw::VideoEncoderConfig::Codec::MJPEG,
      .fps = 10,
    }
  );
  get().writer->record_all();
}

void stop_recording() {
  get().writer->stop();
  get().writer.reset();
}

void shutdown() {
  jr::mw::shutdown();
  if (get().spin_thread.joinable()) {
    get().spin_thread.join();
  };
}
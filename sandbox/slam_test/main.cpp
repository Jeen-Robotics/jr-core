/**
 * Multi-Camera Visual Inertial Odometry (VIO) / SLAM Test
 *
 * Uses all available cameras (0, 1, 2) and fuses their motion estimates.
 *
 * Usage: slam_test [bag_file.mcap] [--single N]
 *   --single N   Use single camera mode with camera N (default: multi-camera)
 */

#include <bag/bag_reader.hpp>
#include <middleware/middleware.hpp>
#include <vio/multi_camera_vio_node.hpp>
#include <vio/multi_camera_visualizer_node.hpp>
#include <vio/vio_node.hpp>
#include <vio/visualizer_node.hpp>

#include <filesystem>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
  std::string bag_path = "camera_test.mcap";
  bool multi_camera = true;
  int single_camera_idx = 0;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--single" || arg == "-s") {
      multi_camera = false;
      if (i + 1 < argc) {
        single_camera_idx = std::stoi(argv[++i]);
      }
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [bag_file.mcap] [--single N]"
                << std::endl;
      std::cout
        << "  bag_file.mcap  Path to bag file (default: camera_test.mcap)"
        << std::endl;
      std::cout << "  --single N     Use single camera mode with camera N"
                << std::endl;
      std::cout << "                 (default: multi-camera using all cameras)"
                << std::endl;
      return 0;
    } else if (arg[0] != '-') {
      bag_path = arg;
    }
  }

  if (!std::filesystem::exists(bag_path)) {
    std::cerr << "Error: Bag file not found: " << bag_path << std::endl;
    std::cerr << "Usage: " << (argc > 0 ? argv[0] : "slam_test")
              << " <bag_file.mcap> [--single N]" << std::endl;
    return 1;
  }

  std::cout << "=== Visual Inertial Odometry Test ===" << std::endl;
  std::cout << "Loading bag: " << bag_path << std::endl;

  if (multi_camera) {
    std::cout << "Mode: Multi-Camera (using cameras 0, 1, 2)" << std::endl;
  } else {
    std::cout << "Mode: Single Camera (using camera " << single_camera_idx
              << ")" << std::endl;
  }

  std::cout << std::endl;
  std::cout << "Controls:" << std::endl;
  std::cout << "  q/ESC - Quit" << std::endl;
  std::cout << "  r     - Reset trajectory visualization" << std::endl;
  std::cout << std::endl;

  // Initialize middleware
  jr::mw::init();

  // Open bag reader
  jr::mw::BagReader bag_reader(bag_path);

  if (multi_camera) {
    // Multi-camera mode
    auto vis_state = std::make_shared<jr::vio::MultiCameraVisState>();
    auto vio_node = std::make_shared<jr::vio::MultiCameraVioNode>(vis_state);
    auto vis_node =
      std::make_shared<jr::vio::MultiCameraVisualizerNode>(vis_state);

    // Spin VIO node and play bag in background thread
    std::thread spin_thread([&]() {
      bag_reader.play(1.0);
      jr::mw::shutdown();
    });

    // Spin visualizer in main thread
    jr::mw::spin(vis_node);

    bag_reader.stop();
    spin_thread.join();
  } else {
    // Single camera mode
    auto vis_state = std::make_shared<jr::vio::VisState>();
    auto vio_node =
      std::make_shared<jr::vio::VioNode>(vis_state, single_camera_idx);
    auto vis_node = std::make_shared<jr::vio::VisualizerNode>(vis_state);

    std::thread spin_thread([&]() {
      bag_reader.play(1.0);
      jr::mw::shutdown();
    });

    jr::mw::spin(vis_node);

    bag_reader.stop();
    spin_thread.join();
  }

  std::cout << "VIO finished." << std::endl;

  return 0;
}

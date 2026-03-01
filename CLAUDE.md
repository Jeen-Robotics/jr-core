# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Desktop (Dev)

```bash
# Install dependencies (Conan 2.x)
scripts/install_deps.sh

# Configure (using preset)
cmake --preset dev

# Build
cmake --build build

# Build with tests enabled
cmake . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DBUILD_TESTS=ON
cmake --build build

# Run all tests
ctest --test-dir build

# Run a single test (example: middleware tests)
./build/core/middleware/test/middleware_test --gtest_filter='MiddlewareTest.PublishSubscribe_DeliversLatest'
```

### Android

```bash
scripts/install_android_deps.sh
cmake --preset android
cmake --build build-android
```

### Formatting

```bash
# Format a file
clang-format -i <file>
```

## Architecture

This is a C++17 robotics framework for multi-camera Visual-Inertial Odometry (VIO) targeting desktop and Android (Camera2 NDK, Sensor framework). It uses a ROS-like pub/sub middleware with protobuf messages.

### Core Modules (`core/`)

- **middleware/** — Pub/sub message bus. `Middleware` is the broker, `Node` is the base class for components, `Publisher<T>` and `Subscription` provide type-safe messaging over protobuf. `spin()` orchestrates node execution. `InProcessMiddleware` is the concrete implementation.
- **nodes/camera_node/** — Publishes camera frames, camera info, and camera pose on `/camera<idx>/frame`, `/camera<idx>/frame/info`, `/camera<idx>/frame/pose`. Uses Pimpl pattern with platform-specific backends.
- **nodes/imu_node/** — Publishes IMU data on `/imu`.
- **vio/** — VIO algorithms. `VioNode` (single-camera) and `MultiCameraVioNode` (multi-camera fusion for cameras 0 and 2, skipping front camera 1). Uses Lucas-Kanade sparse optical flow + Farneback dense flow for depth, with IMU integration. Visualization nodes include `VisualizerNode`, `MultiCameraVisualizerNode`, and `MultiCameraRerunNode` (ReRun SDK, desktop only).
- **bag/** — MCAP-format recording/playback. `BagWriter` records protobuf messages with optional video encoding (H.264/H.265/MJPEG). `BagReader` for playback. Custom `bag.proto` schema defines the format.
- **imgproc/** — Image format conversions (YUV to RGBA).
- **math/** — Mathematical utilities.

### API Layer (`api/`)

C-compatible public API for Android/external consumers: `runner_api.h` (pipeline lifecycle), `camera_device_api.h` (camera management with callbacks), `bag_api.h` (recording control).

### Android Layer (`android/`)

Platform-specific Camera2 NDK and sensor framework implementations.

### Sandbox (`sandbox/`)

Demo/test executables: `camera_test`, `mcap_test`, `slam_test`. Built only when `BUILD_SANDBOX=ON` (desktop only).

## Key Patterns

- **Pub/Sub with protobuf**: All inter-node communication uses typed protobuf messages. Messages from `jr_msgs` package (Conan dependency). Topics follow `/device/data` convention.
- **Node lifecycle**: Nodes inherit `mw::Node`, override `spin_once()`, and are run via `mw::spin()`. Subscriptions auto-unsubscribe via RAII.
- **Pimpl idiom**: Used for platform abstraction (e.g., CameraNode).
- **Thread safety**: `std::mutex` for shared state, `std::atomic` for flags/counters, background threads for message delivery.
- **Conditional compilation**: `#ifdef ANDROID` for platform-specific code. ReRun visualization conditionally linked on desktop.

## Dependencies (Conan)

protobuf 3.21.12, opencv 4.12.0, jr_msgs 1.0.0, mcap 2.1.1, arrow 22.0.0 (ReRun), gtest 1.17.0.

## Commit Style

Prefix with category in brackets: `[new]`, `[enh]`, `[refactor]`, `[fix]`.

#pragma once

namespace jr::mw {

// Video encoder configuration
struct VideoEncoderConfig {
  enum class Codec { H264, H265, MJPEG };

  Codec codec = Codec::H264;
  double fps = 30.0;
};

} // namespace jr::mw
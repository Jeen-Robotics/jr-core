#include "jr_imgproc/imgproc.h"

#include <algorithm>

namespace {

constexpr size_t NUM_RGBA_CHANNELS = 4;
constexpr uint8_t HALF_BYTE = 128;
constexpr uint8_t MAX_BYTE = 255;

template <typename T>
uint8_t clamp(T value) {
  return std::clamp<uint8_t>(value, 0, MAX_BYTE);
}

} // namespace

bool yuv2rgba(const jr_planar_image_t* yuv, jr_image_t* rgba) {
  if (!rgba || !yuv || yuv->num_planes != 3) {
    return false;
  }

  auto* rgba_data = new uint8_t[yuv->width * yuv->height * NUM_RGBA_CHANNELS];

  const auto& y_plane = yuv->planes[0];
  const auto& u_plane = yuv->planes[1];
  const auto& v_plane = yuv->planes[2];

  for (size_t i = 0; i < yuv->height; i++) {
    for (size_t j = 0; j < yuv->width; j++) {
      auto yIndex = i * y_plane.row_stride + j * y_plane.pixel_stride;
      auto uvIndex =
        (i >> 1) * u_plane.row_stride + (j >> 1) * u_plane.pixel_stride;
      auto Y = y_plane.data[yIndex];
      auto U = u_plane.data[uvIndex] - HALF_BYTE;
      auto V = v_plane.data[uvIndex] - HALF_BYTE;

      // YUV to RGB conversion
      auto r = clamp(Y + (1436 * V >> 10));
      auto g = clamp(Y - (352 * U >> 10) - (731 * V >> 10));
      auto b = clamp(Y + (1815 * U >> 10));

      // Store as RGBA
      auto rgbaIndex =
        i * yuv->width * NUM_RGBA_CHANNELS + j * NUM_RGBA_CHANNELS;
      rgba_data[rgbaIndex] = r;
      rgba_data[rgbaIndex + 1] = g;
      rgba_data[rgbaIndex + 2] = b;
      rgba_data[rgbaIndex + 3] = MAX_BYTE; // Alpha channel
    }
  }

  rgba->data = rgba_data;
  rgba->width = yuv->width;
  rgba->height = yuv->height;
  rgba->channels = NUM_RGBA_CHANNELS;

  return true;
}
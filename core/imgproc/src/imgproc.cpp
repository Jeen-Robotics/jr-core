#include "jr_imgproc/imgproc.h"

#include <algorithm>

namespace {

constexpr int32_t NUM_RGBA_CHANNELS = 4;
constexpr uint8_t HALF_BYTE = 128;
constexpr uint8_t MAX_BYTE = 255;

// Fixed-point conversion coefficients (scaled by 2^16 for precision)
constexpr int32_t R_V_COEF = static_cast<int32_t>(1.402f * 65536);
constexpr int32_t G_U_COEF = static_cast<int32_t>(-0.344f * 65536);
constexpr int32_t G_V_COEF = static_cast<int32_t>(-0.714f * 65536);
constexpr int32_t B_U_COEF = static_cast<int32_t>(1.772f * 65536);

constexpr int32_t MIN_INT = 0;
constexpr int32_t MAX_INT = 255;

uint8_t clamp(int32_t value) {
  return static_cast<uint8_t>(std::clamp(value, MIN_INT, MAX_INT));
}

} // namespace

bool yuv2rgba(const jr_planar_image_t* yuv, jr_image_t* rgba) {
  if (!rgba || !yuv || yuv->num_planes != 3) {
    return false;
  }

  const int32_t total_pixels = yuv->width * yuv->height;
  const int32_t total_bytes = total_pixels * NUM_RGBA_CHANNELS;
  
  // Pre-allocate memory
  auto* rgba_data = new uint8_t[total_bytes];
  if (!rgba_data) {
    return false;
  }

  const auto& y_plane = yuv->planes[0];
  const auto& u_plane = yuv->planes[1];
  const auto& v_plane = yuv->planes[2];

  // Process pixels in blocks for better cache utilization
  constexpr int32_t BLOCK_SIZE = 16;
  for (int32_t i = 0; i < yuv->height; i++) {
    for (int32_t j = 0; j < yuv->width; j += BLOCK_SIZE) {
      const int32_t block_width = std::min(BLOCK_SIZE, yuv->width - j);
      
      for (int32_t k = 0; k < block_width; k++) {
        const int32_t x = j + k;
        const int32_t yIndex = i * y_plane.row_stride + x * y_plane.pixel_stride;
        const int32_t uvIndex = (i >> 1) * u_plane.row_stride + (x >> 1) * u_plane.pixel_stride;
        
        const int32_t Y = y_plane.data[yIndex];
        const int32_t U = u_plane.data[uvIndex] - HALF_BYTE;
        const int32_t V = v_plane.data[uvIndex] - HALF_BYTE;

        const int32_t rgbaIndex = (i * yuv->width + x) * NUM_RGBA_CHANNELS;
        
        // Pre-calculate common terms using fixed-point arithmetic
        const int32_t v_term = (V * R_V_COEF) >> 16;
        const int32_t u_term = (U * B_U_COEF) >> 16;
        const int32_t g_term = ((U * G_U_COEF) + (V * G_V_COEF)) >> 16;

        rgba_data[rgbaIndex] = clamp(Y + v_term);
        rgba_data[rgbaIndex + 1] = clamp(Y + g_term);
        rgba_data[rgbaIndex + 2] = clamp(Y + u_term);
        rgba_data[rgbaIndex + 3] = MAX_BYTE;
      }
    }
  }

  rgba->data = rgba_data;
  rgba->width = yuv->width;
  rgba->height = yuv->height;
  rgba->channels = NUM_RGBA_CHANNELS;

  return true;
}
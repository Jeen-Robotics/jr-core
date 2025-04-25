#include <algorithm>
#include <camera/camera_interface.h>

static const int Y_PLANE_BYTES_PER_PIXEL = 1;
static const int U_PLANE_BYTES_PER_PIXEL = 2;

uint8_t *yuv2rgba(const uint8_t *y, const uint8_t *u, const uint8_t *v,
                  int width, int height) {
  uint8_t *rgba = new uint8_t[width * height * 4];

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int yIndex = i * width + j * Y_PLANE_BYTES_PER_PIXEL;
      int uvIndex = (i >> 1) * width + (j >> 1) * U_PLANE_BYTES_PER_PIXEL;
      int Y = y[yIndex];
      int U = u[uvIndex] - 128;
      int V = v[uvIndex] - 128;

      // YUV to RGB conversion
      uint8_t r = std::clamp(Y + (1436 * V >> 10), 0, 255);
      uint8_t g = std::clamp(Y - (352 * U >> 10) - (731 * V >> 10), 0, 255);
      uint8_t b = std::clamp(Y + (1815 * U >> 10), 0, 255);

      // Store as RGBA
      int rgbaIndex = i * width * 4 + j * 4;
      rgba[rgbaIndex] = r;
      rgba[rgbaIndex + 1] = g;
      rgba[rgbaIndex + 2] = b;
      rgba[rgbaIndex + 3] = 255; // Alpha channel
    }
  }

  return rgba;
}
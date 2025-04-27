#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t width;
  int32_t height;
  int32_t channels;
  uint8_t* data;
} jr_image_t;

void jr_image_destroy(jr_image_t* image);

typedef struct {
  uint8_t* data;
  int32_t row_stride;
  int32_t pixel_stride;
} jr_plane_t;

void jr_plane_destroy(jr_plane_t* plane);

typedef struct {
  int32_t width;
  int32_t height;
  int32_t num_planes;
  jr_plane_t* planes;
} jr_planar_image_t;

void jr_planar_image_destroy(jr_planar_image_t* planar_image);

#ifdef __cplusplus
}
#endif
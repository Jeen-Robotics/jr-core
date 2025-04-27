#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t* yuv2rgba(
  const uint8_t* y,
  const uint8_t* u,
  const uint8_t* v,
  int width,
  int height
);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_INTERFACE_H
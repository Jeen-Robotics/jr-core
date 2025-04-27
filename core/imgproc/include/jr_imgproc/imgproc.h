#pragma once

#include <stdbool.h>

#include "image.h"

#ifdef __cplusplus
extern "C" {
#endif

bool yuv2rgba(const jr_planar_image_t* yuv, jr_image_t* rgba);

#ifdef __cplusplus
}
#endif
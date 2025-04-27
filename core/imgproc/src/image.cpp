#include "jr_imgproc/image.h"

void jr_image_destroy(jr_image_t* image) {
  delete[] image->data;
}

void jr_plane_destroy(jr_plane_t* plane) {
  delete[] plane->data;
}

void jr_planar_image_destroy(jr_planar_image_t* planar_image) {
  for (size_t i = 0; i < planar_image->num_planes; i++) {
    jr_plane_destroy(&planar_image->planes[i]);
  }
  delete[] planar_image->planes;
}
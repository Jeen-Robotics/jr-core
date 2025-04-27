#include "jr_imgproc/image.h"

void jr_image_destroy(jr_image_t* image) {
  if (image && image->data) {
    delete[] image->data;
  }
}

void jr_plane_destroy(jr_plane_t* plane) {
  if (plane && plane->data) {
    delete[] plane->data;
  }
}

void jr_planar_image_destroy(jr_planar_image_t* planar_image) {
  if (planar_image && planar_image->planes) {
    for (auto i = 0; i < planar_image->num_planes; i++) {
      jr_plane_destroy(&planar_image->planes[i]);
    }
    delete[] planar_image->planes;
  }
}
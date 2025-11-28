#include "calibration_extractor.hpp"

#ifdef __ANDROID__
#include <android/log.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#endif

namespace jr {

CalibrationExtractor::CalibrationExtractor(int camera_idx)
    : camera_idx_(camera_idx) {
}

std::optional<sensor_msgs::CameraInfo> CalibrationExtractor::extract(
  int width,
  int height
) const {
#ifdef __ANDROID__
  ACameraManager* manager = ACameraManager_create();
  if (!manager) {
    return std::nullopt;
  }

  ACameraIdList* camera_ids = nullptr;
  if (ACameraManager_getCameraIdList(manager, &camera_ids) != ACAMERA_OK) {
    ACameraManager_delete(manager);
    return std::nullopt;
  }

  if (camera_idx_ < 0 || camera_idx_ >= camera_ids->numCameras) {
    ACameraManager_deleteCameraIdList(camera_ids);
    ACameraManager_delete(manager);
    return std::nullopt;
  }

  const char* cam_id = camera_ids->cameraIds[camera_idx_];
  ACameraMetadata* metadata = nullptr;
  if (ACameraManager_getCameraCharacteristics(manager, cam_id, &metadata) !=
      ACAMERA_OK) {
    ACameraManager_deleteCameraIdList(camera_ids);
    ACameraManager_delete(manager);
    return std::nullopt;
  }

  sensor_msgs::CameraInfo info;
  info.set_width(width);
  info.set_height(height);
  info.set_distortion_model("plumb_bob");

  ACameraMetadata_const_entry entry;

  // Intrinsics: [f_x, f_y, c_x, c_y, s]
  if (ACameraMetadata_getConstEntry(
        metadata,
        ACAMERA_LENS_INTRINSIC_CALIBRATION,
        &entry
      ) == ACAMERA_OK) {
    float fx = entry.data.f[0];
    float fy = entry.data.f[1];
    float cx = entry.data.f[2];
    float cy = entry.data.f[3];
    float s = entry.data.f[4];

    // K = [fx, s, cx, 0, fy, cy, 0, 0, 1]
    // Row 1
    info.add_k(fx);
    info.add_k(s);
    info.add_k(cx);
    // Row 2
    info.add_k(0);
    info.add_k(fy);
    info.add_k(cy);
    // Row 3
    info.add_k(0);
    info.add_k(0);
    info.add_k(1);

    // P = [K|0] for unrectified/raw images (or if we assume identity R)
    // Row 1
    info.add_p(fx);
    info.add_p(s);
    info.add_p(cx);
    info.add_p(0);
    // Row 2
    info.add_p(0);
    info.add_p(fy);
    info.add_p(cy);
    info.add_p(0);
    // Row 3
    info.add_p(0);
    info.add_p(0);
    info.add_p(1);
    info.add_p(0);
  }

  // Distortion: Android [k1, k2, k3, k4, k5]
  // k4=p1, k5=p2.
  // ROS plumb_bob D: [k1, k2, p1, p2, k3]
  if (ACameraMetadata_getConstEntry(
        metadata,
        ACAMERA_LENS_DISTORTION,
        &entry
      ) == ACAMERA_OK) {
    float k1 = entry.data.f[0];
    float k2 = entry.data.f[1];
    float k3 = entry.data.f[2];
    float k4 = entry.data.f[3]; // p1
    float k5 = entry.data.f[4]; // p2

    info.add_d(k1);
    info.add_d(k2);
    info.add_d(k4);
    info.add_d(k5);
    info.add_d(k3);
  }

  // R = Identity
  info.add_r(1);
  info.add_r(0);
  info.add_r(0);
  info.add_r(0);
  info.add_r(1);
  info.add_r(0);
  info.add_r(0);
  info.add_r(0);
  info.add_r(1);

  ACameraMetadata_free(metadata);
  ACameraManager_deleteCameraIdList(camera_ids);
  ACameraManager_delete(manager);

  return info;
#else
  (void)width;
  (void)height;
  return std::nullopt;
#endif
}

std::optional<geometry_msgs::PoseStamped> CalibrationExtractor::extract_pose(
) const {
#ifdef __ANDROID__
  ACameraManager* manager = ACameraManager_create();
  if (!manager) {
    return std::nullopt;
  }

  ACameraIdList* camera_ids = nullptr;
  if (ACameraManager_getCameraIdList(manager, &camera_ids) != ACAMERA_OK) {
    ACameraManager_delete(manager);
    return std::nullopt;
  }

  if (camera_idx_ < 0 || camera_idx_ >= camera_ids->numCameras) {
    ACameraManager_deleteCameraIdList(camera_ids);
    ACameraManager_delete(manager);
    return std::nullopt;
  }

  const char* cam_id = camera_ids->cameraIds[camera_idx_];
  ACameraMetadata* metadata = nullptr;
  if (ACameraManager_getCameraCharacteristics(manager, cam_id, &metadata) !=
      ACAMERA_OK) {
    ACameraManager_deleteCameraIdList(camera_ids);
    ACameraManager_delete(manager);
    return std::nullopt;
  }

  geometry_msgs::PoseStamped pose_msg;
  bool pose_found = false;
  ACameraMetadata_const_entry entry;

  if (ACameraMetadata_getConstEntry(
        metadata,
        ACAMERA_LENS_POSE_TRANSLATION,
        &entry
      ) == ACAMERA_OK) {
    pose_msg.mutable_pose()->mutable_position()->set_x(entry.data.f[0]);
    pose_msg.mutable_pose()->mutable_position()->set_y(entry.data.f[1]);
    pose_msg.mutable_pose()->mutable_position()->set_z(entry.data.f[2]);
    pose_found = true;
  }

  if (ACameraMetadata_getConstEntry(
        metadata,
        ACAMERA_LENS_POSE_ROTATION,
        &entry
      ) == ACAMERA_OK) {
    // Android rotation is x, y, z, w
    pose_msg.mutable_pose()->mutable_orientation()->set_x(entry.data.f[0]);
    pose_msg.mutable_pose()->mutable_orientation()->set_y(entry.data.f[1]);
    pose_msg.mutable_pose()->mutable_orientation()->set_z(entry.data.f[2]);
    pose_msg.mutable_pose()->mutable_orientation()->set_w(entry.data.f[3]);
    pose_found = true;
  }

  ACameraMetadata_free(metadata);
  ACameraManager_deleteCameraIdList(camera_ids);
  ACameraManager_delete(manager);

  if (pose_found) {
    return pose_msg;
  }
  return std::nullopt;

#else
  return std::nullopt;
#endif
}

} // namespace jr

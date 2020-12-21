// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#include "mapping/keyframe_updater.h"

#include "utils/common/math.h"

namespace long_term_relocalization {
namespace mapping {

KeyframeUpdater::KeyframeUpdater(double keyframe_delta_trans,
                                 double keyframe_delta_angle)
    : keyframe_delta_translation_(keyframe_delta_trans),
      keyframe_delta_rotation_angle_(common::DegToRad(keyframe_delta_angle)),
      is_initialized_(true), accumulate_distance_(0.0),
      prev_keypose_(transform::Rigid3d::Identity()) {}

bool KeyframeUpdater::UpdatePose(const transform::Rigid3d &pose) {
  // first frame is always registered to the graph
  if (is_initialized_) {
    is_initialized_ = false;
    prev_keypose_ = pose;
    return true;
  }

  // calculate the delta_pose transformation from the previous keyframe_
  const transform::Rigid3d delta_pose = prev_keypose_.inverse() * pose;
  const double delt_translation = delta_pose.translation().norm();
  const double delt_angle = std::acos(
      Eigen::Quaterniond(
          Eigen::Isometry3d(transform::Rigid3dToMatrix4d(delta_pose)).linear())
          .w());

  // too close to the previous frame
  if (delt_translation < keyframe_delta_translation_ &&
      delt_angle < keyframe_delta_rotation_angle_) {
    return false;
  }

  accumulate_distance_ += delt_translation;
  prev_keypose_ = pose;
  return true;
}

} // namespace mapping
} // namespace long_term_relocalization

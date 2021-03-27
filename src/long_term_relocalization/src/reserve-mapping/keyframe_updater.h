// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#pragma once

#include <cmath>

#include "kindr/minimal/quat-transformation.h"

namespace long_term_relocalization {
namespace mapping {

// Decide whether a new frame should be registered to the map according to the
// pose of the point cloud frame.
class KeyframeUpdater {

public:
  KeyframeUpdater(double keyframe_delta_trans /*meter*/,
                  double keyframe_delta_angle /*degree*/);

  virtual ~KeyframeUpdater() = default;

  // Return true if the frame should be registered
  bool UpdatePose(const kindr::minimal::QuatTransformation &pose);

  // The last keyframe's accumulated distance from the first keyframe.
  double get_accumulate_distance() const { return accumulate_distance_; }

private:
  double keyframe_delta_translation_ = 0.0;
  double keyframe_delta_rotation_angle_ = 0.0;

  bool is_initialized_ = false;
  double accumulate_distance_ = 0.0;
  kindr::minimal::QuatTransformation prev_keypose_;
};

} // namespace mapping
} // namespace long_term_relocalization

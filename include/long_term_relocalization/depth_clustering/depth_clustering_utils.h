// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-1.

#pragma once

#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include "common/fixed_array2d.h"
#include "common/math/math.h"

namespace long_term_relocalization {

struct Index2D {
  Index2D() = default;
  Index2D(int row, int col) : row(row), col(col) {}
  int row = 0;
  int col = 0;
};

enum class PointLabel {
  kNone = 0,
  kUnProcessed = 1,
  kGround = 2,
  kNonGround = 3,
  kCluster = 4,
};

int WrapCircularIndex(int x, int width);

cv::Mat RenderRangeImage(const common::FixedArray2D<double> &range_image);

} // namespace long_term_relocalization

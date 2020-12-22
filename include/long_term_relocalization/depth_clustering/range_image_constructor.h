// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-10-09.

#pragma once

#include <cmath>
#include <map>

#include <glog/logging.h>

#include "common/fixed_array2d.h"
#include "common/math/math.h"
#include "common/pcl_utils/pcl_utils.h"
#include "long_term_relocalization/depth_clustering/depth_clustering_utils.h"
#include "long_term_relocalization/utils/params_types.h"

namespace long_term_relocalization {

// Note: this class is designed for rs80 lidar.
template <typename PointT /*MUST contain ring field*/> class RangeImageConstructor {
  using PointCloud = pcl::PointCloud<PointT>;

public:
  struct Options {
    int width = 0;
    int height = 0;
    std::vector<double> vert_angles; // degree
    std::vector<double> hori_angles; // degree
  };

  RangeImageConstructor(const Options &options) : width_(options.width), height_(options.height) {
    points_image_ =
        common::FixedArray2D<std::vector<PointT>>(height_, width_, std::vector<PointT>());
    range_image_ = common::FixedArray2D<double>(height_, width_, 0);
    label_image_ = common::FixedArray2D<PointLabel>(height_, width_, PointLabel::kNone);

    SortBeamTable(options.vert_angles);
    CalculateColOffset(options.hori_angles);
  }

  virtual ~RangeImageConstructor() = default;

  void Process(const PointCloud &cloud) {
    ResetParams();
    ConstructRangeImage(cloud);
  }

  const common::FixedArray2D<double> &get_range_image() const { return range_image_; }
  const common::FixedArray2D<std::vector<PointT>> &get_points_image() const {
    return points_image_;
  }
  const common::FixedArray2D<PointLabel> &get_label_image() const { return label_image_; }
  const std::vector<Index2D> &get_points_indices_map() const { return points_indices_map_; }

private:
  void SortBeamTable(const std::vector<double> &vert_angles) {
    CHECK_EQ(vert_angles.size(), height_);

    beam_ring_table_.resize(height_);
    std::vector<int> sorted_idx(height_);
    std::iota(sorted_idx.begin(), sorted_idx.end(), 0);

    // Decrease. The heightest laser beam is the first row.
    std::sort(sorted_idx.begin(), sorted_idx.end(), [&vert_angles](int lhs, int rhs) -> bool {
      return vert_angles[lhs] > vert_angles[rhs];
    });

    for (int i = 0; i < height_; i++) {
      beam_ring_table_[sorted_idx[i]] = i;
    }
  }

  // TODO(silin): currently, we didn't use the col offset.
  // We use projection method to calculate column value for each 3d point.
  void CalculateColOffset(const std::vector<double> &hori_angles) {
    CHECK_EQ(hori_angles.size(), height_);
    col_offset_.resize(height_);
    const double hori_resolution = 360.0 / width_;

    int base_row = -1;
    for (int row = 0; row < height_; ++row) {
      if (beam_ring_table_[row] == 0) {
        base_row = row;
        break;
      }
    }

    CHECK_GE(base_row, 0);
    for (int fake_row = 0; fake_row < height_; ++fake_row) {
      const int real_row = beam_ring_table_[fake_row];
      col_offset_[real_row] =
          std::round<int>((hori_angles[fake_row] - hori_angles[base_row]) / hori_resolution);
    }
  }

  // Construct range image by projection.
  void ConstructRangeImage(const PointCloud &cloud) {
    points_indices_map_.resize(cloud.size(), Index2D(-1, -1));

    for (int i = 0; i < cloud.size(); ++i) {
      const auto &point = cloud.points[i];
      if (!pcl::isFinite(point)) {
        continue;
      }

      const int real_row = height_ - point.ring - 1;

      double hori_angle = std::atan2(point.y, point.x); //[-PI, +PI]
      if (hori_angle < 0) {
        hori_angle += 2 * M_PI; //[0, 2*PI]
      }
      const int col = std::round<int>(width_ * 0.5 * hori_angle / M_PI);
      const int real_col = WrapCircularIndex(col, width_);

      const double range = pcl_utils::PointLength(point);
      if (range < math::kEpsilon) {
        continue;
      }

      if (points_image_(real_row, real_col).empty()) {
        points_image_(real_row, real_col).push_back(point);
        range_image_(real_row, real_col) = range;
      } else {
        // Find a more close point. Update first point and range image.
        if (range_image_(real_row, real_col) > range) {
          range_image_(real_row, real_col) = range;
          points_image_(real_row, real_col).front() = point;
        }
      }
      points_image_(real_row, real_col).push_back(point);
      label_image_(real_row, real_col) = PointLabel::kUnProcessed;
      points_indices_map_[i] = Index2D(real_row, real_col);
    }

    // Use col offset to construct range image.
    // for (int row = 0; row < height_; ++row) {
    //   for (int col = 0; col < width_; ++col) {
    //     const auto &point = cloud.at(col, row);
    //     if (!pcl::isFinite(point)) {
    //       continue;
    //     }
    //     const int real_row = height_ - point.ring - 1;
    //     double hori_angle = std::atan2(point.y, point.x); //[-PI, +PI]
    //     if (hori_angle < 0) {
    //       hori_angle += 2 * M_PI; //[0, 2*PI]
    //     }
    //     const int tmp_col = std::round<int>(width_ * 0.5 * hori_angle / M_PI);
    //     const int estimate_col = WrapCircularIndex(tmp_col, width_);
    //     const int real_col = WrapCircularIndex(col + col_offset_[real_row], width_);
    //     // LOG(INFO) << estimate_col << " " << real_col << " " << col_offset_[real_row];

    //     range_image_(real_row, real_col) = pcl_utils::PointLength(point);
    //     if (range_image_(real_row, real_col) > math::kEpsilon) {
    //       points_image_(real_row, real_col) = point;
    //       label_image_(real_row, real_col) = PointLabel::kUnProcessed;
    //     }
    //     points_indices_map_[row * width_ + col] = Index2D(real_row, real_col);
    //   }
    // }
  }

  void ResetParams() {
    for (auto it = points_image_.begin(); it != points_image_.end(); ++it) {
      it->clear();
    }
    std::fill(range_image_.begin(), range_image_.end(), 0);
    std::fill(label_image_.begin(), label_image_.end(), PointLabel::kNone);
    points_indices_map_.clear();
  }

  int width_;
  int height_;

  //      <0>         | < 1 ...... n >
  // <closest_point>  | < all points >
  common::FixedArray2D<std::vector<PointT>> points_image_;
  common::FixedArray2D<double> range_image_;
  common::FixedArray2D<PointLabel> label_image_;

  std::vector<Index2D> points_indices_map_;

  std::vector<int> beam_ring_table_; // beam_ring_table_[fake_row] = real row
  std::vector<int> col_offset_;      // col_offset_[real_row] = real col offset
};

} // namespace long_term_relocalization

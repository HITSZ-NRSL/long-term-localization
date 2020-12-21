// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-23.
// Author: supengcc@163.com(Alex Su)

#pragma once

#include <iostream>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>

#include <glog/logging.h>

#include <pcl/common/common.h>
#include <pcl/filters/impl/filter.hpp>

#include "depth_clustering/depth_clustering_utils.h"
#include "depth_clustering/ground_plane_fitter.h"
#include "depth_clustering/range_image_constructor.h"
#include "utils/common/fixed_array2d.h"
#include "utils/common/math.h"
#include "utils/common/pcl_utils.h"
#include "utils/common/tic_toc.h"
#include "utils/params/params_types.h"

namespace long_term_relocalization {

namespace {
const std::vector<Index2D> kNeighbors = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
}

// Note: this class is designed for velodyne32 now.
template <typename PointT> class DepthClustering {
  using PointCloud = pcl::PointCloud<PointT>;

public:
  explicit DepthClustering(const params::DepthClusteringParams &params) : params_(params) {
    Initialize();
  }
  virtual ~DepthClustering() = default;

  void Process(const PointCloud &cloud_in) {
    ResetParams();

    PointCloud cloud;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud_in, cloud, indices);

    range_image_constructor_->Process(cloud);
    FindGroundPoints(cloud);
    FindClusterPoints();
  }

  PointCloud ExtractGroundPoints() {
    PointCloud cloud;
    const auto &points_image = range_image_constructor_->get_points_image();
    for (const auto &index : ground_indices_) {
      const auto &points = points_image(index.row, index.col);
      for (const auto &point : points) {
        cloud.push_back(point);
      }
    }
    return cloud;
  }

  PointCloud ExtractClusterPoints() {
    std::vector<typename PointCloud::Ptr> clusters = this->ExtractClusters();

    PointCloud cloud;
    for (const auto &cluster : clusters) {
      cloud += *cluster;
    }

    return cloud;
  }

  std::vector<typename PointCloud::Ptr> ExtractClusters() {
    std::vector<typename PointCloud::Ptr> cloud;
    cloud.reserve(500);
    const auto &points_image = range_image_constructor_->get_points_image();
    for (const auto &item : clusters_indices_) {
      const int intensity = std::rand() % 255;
      typename PointCloud::Ptr cluster_cloud(new PointCloud());
      for (const auto &index : item.second) {
        const auto &points = points_image(index.row, index.col);
        for (const auto &point : points) {
          cluster_cloud->push_back(point);
          cluster_cloud->back().intensity = intensity;
        }
      }
      if (IsGoodCluster(*cluster_cloud)) {
        cloud.push_back(cluster_cloud);
      }
    }
    return cloud;
  }

private:
  void Initialize() {
    width_ = params_.width;
    height_ = params_.height;

    range_image_constructor_ = std::make_unique<RangeImageConstructor<PointT>>(
        typename RangeImageConstructor<PointT>::Options{.width = params_.width,
                                                        .height = params_.height,
                                                        .vert_angles = params_.vert_angles,
                                                        .hori_angles = params_.hori_angles});
    hori_resolution_ = 2 * M_PI / width_;
    sin_diff_hori_ = std::sin(hori_resolution_);
    cos_diff_hori_ = std::cos(hori_resolution_);
    std::vector<double> sorted_vert_angles(params_.vert_angles);
    std::sort(sorted_vert_angles.begin(), sorted_vert_angles.end(), std::greater<double>());
    for (const double &vert_angle : sorted_vert_angles) {
      sin_verts_.push_back(std::sin(common::DegToRad(vert_angle)));
      cos_verts_.push_back(std::cos(common::DegToRad(vert_angle)));
    }
    CHECK_EQ(sin_verts_.size(), height_);
    for (int i = 1; i < sorted_vert_angles.size(); ++i) {
      const double diff_angle =
          common::DegToRad(std::abs(sorted_vert_angles[i] - sorted_vert_angles[i - 1]));
      sin_diff_verts_.push_back(std::sin(diff_angle));
      cos_diff_verts_.push_back(std::cos(diff_angle));
    }
    ground_plane_fitter_ =
        std::make_unique<GroundPlaneFitter<PointT>>(params_.ground_plane_fitter_params);
    label_image_ = common::FixedArray2D<PointLabel>(height_, width_, PointLabel::kNone);
  }

  void FindGroundPoints(const PointCloud &cloud) {
    const auto &range_image = range_image_constructor_->get_range_image();

    // show range image
    // cv::imshow("range image", RenderRangeImage(range_image));
    // cv::waitKey(1);

    // Calculate angle image.
    common::FixedArray2D<double> angle_image(height_, width_, std::numeric_limits<double>::max());
    for (int row = 1; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        const double r1 = range_image(row - 1, col);
        const double r2 = range_image(row, col);
        const double diff_z = std::abs(r1 * sin_verts_[row - 1] - r2 * sin_verts_[row]);
        const double diff_xy = std::abs(r1 * cos_verts_[row - 1] - r2 * cos_verts_[row]);
        angle_image(row, col) = std::atan2(diff_z, diff_xy);
      }
    }

    // Update label image in current frame.
    const auto &label_image = range_image_constructor_->get_label_image();
    std::copy(label_image.begin(), label_image.end(), label_image_.begin());

    // Choose ground seeds point from semantic and ground plane fitter.
    const std::vector<Index2D> ground_seed_indices = FindGroundSeedIndices(cloud, angle_image);
    for (const Index2D &index : ground_seed_indices) {
      if (!IsPointUnProcessed(index)) {
        continue;
      }
      LabelGroundPoints(angle_image, index);
    }
  }

  std::vector<Index2D> FindGroundSeedIndices(const PointCloud &cloud,
                                             const common::FixedArray2D<double> &angle_image) {
    std::vector<Index2D> ground_seed_indices;

    pcl::PointIndices::Ptr gnds_indices(new pcl::PointIndices);
    pcl::PointIndices::Ptr ngnds_indices(new pcl::PointIndices);
    ground_plane_fitter_->Process(cloud, gnds_indices, ngnds_indices);

    const std::vector<Index2D> &points_indices_map =
        range_image_constructor_->get_points_indices_map();
    for (int i = 0; i < gnds_indices->indices.size(); ++i) {
      // if (cloud.points[gnds_indices->indices[i]].label != params_.ground_semantic_label) {
      //   continue;
      // }
      const Index2D &index = points_indices_map[gnds_indices->indices[i]];
      if (angle_image(index.row, index.col) < params_.max_ground_seed_angle) {
        ground_seed_indices.push_back(index);
      }
    }

    return ground_seed_indices;
  }

  void LabelGroundPoints(const common::FixedArray2D<double> &angle_image, const Index2D &index) {
    // bfs
    std::queue<Index2D> q;
    q.push(index);
    label_image_(index.row, index.col) = PointLabel::kGround;
    ground_indices_.push_back(index);

    const auto &points_image = range_image_constructor_->get_points_image();

    while (!q.empty()) {
      const Index2D &cur_index = q.front();
      q.pop();
      for (const Index2D &neighbor : kNeighbors) {
        Index2D neighbor_index(cur_index.row + neighbor.row, cur_index.col + neighbor.col);
        neighbor_index.col = WrapCircularIndex(neighbor_index.col, width_);

        if (!IsIndexValid(neighbor_index) || !IsPointUnProcessed(neighbor_index)) {
          continue;
        }

        const double diff_z =
            std::abs(points_image(cur_index.row, cur_index.col).front().z -
                     points_image(neighbor_index.row, neighbor_index.col).front().z);
        const double diff_angle = std::abs(angle_image(cur_index.row, cur_index.col) -
                                           angle_image(neighbor_index.row, neighbor_index.col));
        if (diff_angle < params_.max_ground_diff_angle && diff_z < params_.max_ground_diff_z) {
          q.push(neighbor_index);
          label_image_(neighbor_index.row, neighbor_index.col) = PointLabel::kGround;
          ground_indices_.push_back(neighbor_index);
        }
      }
    }
  }

  bool IsPointUnProcessed(const Index2D &index) const {
    return label_image_(index.row, index.col) == PointLabel::kUnProcessed;
  }

  bool IsIndexValid(const Index2D &index) const {
    return index.col >= 0 && index.col < width_ && index.row >= 0 && index.row < height_;
  }

  void FindClusterPoints() {
    int cluster_id = 0;
    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        const Index2D index(row, col);
        if (!IsPointUnProcessed(index)) {
          continue;
        }
        LabelClusterPoints(index, cluster_id);
        ++cluster_id;
      }
    }
  }

  void LabelClusterPoints(const Index2D &index, int cluster_id) {
    std::queue<Index2D> q;
    q.push(index);
    label_image_(index.row, index.col) = PointLabel::kCluster;
    clusters_indices_[cluster_id].push_back(index);

    while (!q.empty()) {
      auto cur_index = q.front();
      q.pop();
      for (const Index2D &neighbor : kNeighbors) {
        Index2D neighbor_index(cur_index.row + neighbor.row, cur_index.col + neighbor.col);
        neighbor_index.col = WrapCircularIndex(neighbor_index.col, width_);

        if (!IsIndexValid(neighbor_index) || !IsPointUnProcessed(neighbor_index)) {
          continue;
        }

        if (AreTwoPointsInSameCluster(neighbor_index, cur_index)) {
          q.push(neighbor_index);
          label_image_(neighbor_index.row, neighbor_index.col) = PointLabel::kCluster;
          clusters_indices_[cluster_id].push_back(neighbor_index);
        }
      }
    }
  }

  bool AreTwoPointsInSameCluster(const Index2D &left, const Index2D &right) {
    const auto &points_image = range_image_constructor_->get_points_image();
    const PointT &point_left = points_image(left.row, left.col).front();
    const PointT &point_right = points_image(right.row, right.col).front();

    const float distance = common::DistanceBetweenTwoPoints(point_left, point_right);
    return distance < params_.max_cluster_distance;

    if (distance < params_.max_cluster_distance) {
      return true;
    } else {
      // only judge 4-neibourhood
      return IsGeometryConditionSatisfied(left, right);
    }
  }

  bool IsGeometryConditionSatisfied(const Index2D &left, const Index2D &right) {
    const auto &range_image = range_image_constructor_->get_range_image();
    // judge Vertical or horizon
    int horizon_diff = std::abs(left.row - right.row);
    int vertical_diff = std::abs(left.col - right.col);
    double sin_psi, cos_psi = 0;
    if (horizon_diff == 0 && vertical_diff == 1) {
      sin_psi = sin_diff_hori_; // horizon resolution
      cos_psi = cos_diff_hori_;
    } else if (horizon_diff == 1 && vertical_diff == 0) {
      int psi_row = std::min(left.row, right.row);
      sin_psi = sin_diff_verts_[psi_row]; // vertical resolution
      cos_psi = cos_diff_verts_[psi_row];
    } else {
      // not lebong to 4-neighbourhood.
      std::tie(sin_psi, cos_psi) = CalculatePsi(left, right);
    }

    const double d1 = std::max(range_image(left.row, left.col), range_image(right.row, right.col));
    const double d2 = std::min(range_image(left.row, left.col), range_image(right.row, right.col));
    const double theta = std::atan((d2 * sin_psi) / (d1 - d2 * cos_psi));
    return theta > params_.min_cluster_angle;
  }

  std::pair<double, double> CalculatePsi(const Index2D &left, const Index2D &right) {
    const auto &points_image = range_image_constructor_->get_points_image();
    const PointT &point_left = points_image(left.row, left.col).front();
    const PointT &point_right = points_image(right.row, right.col).front();
    Eigen::Vector3d point_left_vec(point_left.x, point_left.y, point_left.z);
    Eigen::Vector3d point_right_vec(point_right.x, point_right.y, point_right.z);
    double radian_psi = atan2(point_left_vec.cross(point_right_vec).norm(),
                              point_left_vec.transpose() * point_right_vec); // rad
    if (point_left_vec.cross(point_right_vec).z() < 0) {
      radian_psi = 2 * M_PI - radian_psi;
    }
    return {std::sin(radian_psi), std::cos(radian_psi)};
  }

  void ResetParams() {
    clusters_indices_.clear();
    ground_indices_.clear();
    ground_indices_.reserve(width_ * height_);
    std::fill(label_image_.begin(), label_image_.end(), PointLabel::kNone);
  }

  bool IsGoodCluster(const PointCloud &cluster_cloud) {
    PointT min_pt, max_pt;
    pcl::getMinMax3D(cluster_cloud, min_pt, max_pt);
    return (max_pt.z - min_pt.z > params_.min_good_cluster_diff_z) &&
           (cluster_cloud.size() > params_.min_cluster_size);
  }

  params::DepthClusteringParams params_;
  int width_;
  int height_;
  std::unique_ptr<RangeImageConstructor<PointT>> range_image_constructor_;

  std::unordered_map<int /*cluster id*/, std::vector<Index2D>> clusters_indices_;
  std::vector<Index2D> ground_indices_;

  double hori_resolution_ = 0.0;
  double sin_diff_hori_ = 0.0;
  double cos_diff_hori_ = 0.0;

  std::vector<double> sin_verts_, sin_diff_verts_;
  std::vector<double> cos_verts_, cos_diff_verts_;

  std::unique_ptr<GroundPlaneFitter<PointT>> ground_plane_fitter_;

  common::FixedArray2D<PointLabel> label_image_;
};

} // namespace long_term_relocalization

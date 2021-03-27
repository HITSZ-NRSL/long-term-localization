// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-22.

#pragma once

#include <cmath>

#include "common/pcl_utils/pcl_types.h"

namespace pcl_utils {

template <typename PointT> double PointLength(const PointT &point) {
  static_assert(pcl::traits::has_xyz<PointT>::value,
                "PointT must be a structure containing XYZ coordinates");
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

template <typename PointT> bool PointZAxisCompare(const PointT &lhs, const PointT &rhs) {
  static_assert(pcl::traits::has_xyz<PointT>::value,
                "PointT must be a structure containing XYZ coordinates");
  return lhs.z < rhs.z;
}

void ExtractSemanticPointsIndices(const pcl_utils::PointIRLCloud &cloud, int label,
                                  std::vector<int> *indices);

void ExtractSemanticPoints(const pcl_utils::PointIRLCloud &cloud, int label,
                           pcl_utils::PointICloud::Ptr cloud_out);

template <typename PointT> PointT Vector4fToPoint3d(const Eigen::Vector4f &vec) {
  PointT pt;
  pt.x = vec.x();
  pt.y = vec.y();
  pt.z = vec.z();

  return pt;
}

pcl::PointXY Vector4fToPoint2d(const Eigen::Vector4f &vec);

template <typename PointT> double ComputeXYDiameter(const PointT &point1, const PointT &point2) {
  const double diff_x = point1.x - point2.x;
  const double diff_y = point2.y - point2.y;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

template <typename PointCloudT>
void TranslateCloud(const Eigen::Vector3d &translation, PointCloudT *cloud) {
  for (size_t i = 0u; i < cloud->size(); ++i) {
    cloud->points[i].x += translation.x();
    cloud->points[i].y += translation.y();
    cloud->points[i].z += translation.z();
  }
}

template <typename PointT> float DistanceBetweenTwoPoints(const PointT &p1, const PointT &p2) {
  const float diff_x = p1.x - p2.x;
  const float diff_y = p1.y - p2.y;
  const float diff_z = p1.z - p2.z;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}
} // namespace pcl_utils

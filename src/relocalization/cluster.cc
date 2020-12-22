// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/21.

#include "long_term_relocalization/relocalization/cluster.h"

#include <numeric>

#include "common/math/math.h"
#include "long_term_relocalization/relocalization/cluster_manager.h"

namespace long_term_relocalization {

//============================================================
// Cluster Implementation
//============================================================
Cluster::Cluster(const PointCloud::Ptr &cloud, int id, ClustersManager *clusters_manager,
                 int times_of_observed)
    : cloud_(cloud), id_(id), clusters_manager_(clusters_manager),
      times_of_observed_(times_of_observed) {
  CHECK(cloud_ != nullptr && !cloud_->empty());

  Eigen::Vector4f min_vec, max_vec;
  pcl::getMinMax3D(*cloud_, min_vec, max_vec);
  min_point_ = pcl_utils::Vector4fToPoint3d<PointT>(min_vec);
  max_point_ = pcl_utils::Vector4fToPoint3d<PointT>(max_vec);
  diameter_ = pcl_utils::ComputeXYDiameter(min_point_, max_point_);
  label_ = static_cast<int>(cloud_->front().label);

  ResetClusterCloudIntensity();
}

void Cluster::MergeCloud(const PointCloud::Ptr &cloud_in) {
  CHECK(cloud_in != nullptr && !cloud_in->empty());
  // CHECK_EQ(label_, cloud_in->front().label);

  // Update centroid first.
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*cloud_in, centroid_in);
  const int num_points = this->size() + cloud_in->size();
  const float x =
      (this->centroid().x * this->size() + centroid_in.x() * cloud_in->size()) / num_points;
  const float y =
      (this->centroid().y * this->size() + centroid_in.y() * cloud_in->size()) / num_points;
  const float z =
      (this->centroid().z * this->size() + centroid_in.z() * cloud_in->size()) / num_points;

  // LOG(INFO) << "Diff centroid: " << this->centroid().x - x << " " << this->centroid().y - y << "
  // "
  //           << this->centroid().z - z;

  this->mutable_centroid()->x = x;
  this->mutable_centroid()->y = y;
  this->mutable_centroid()->z = z;
  this->mutable_centroid2d()->x = x;
  this->mutable_centroid2d()->y = y;

  ResetMergeCloudIntensity(cloud_in); // For visualization.
  *cloud_ += *cloud_in;

  Eigen::Vector4f min_vec, max_vec;
  pcl::getMinMax3D(*cloud_in, min_vec, max_vec);
  min_point_.x = std::min(min_point_.x, min_vec.x());
  min_point_.y = std::min(min_point_.y, min_vec.y());
  min_point_.z = std::min(min_point_.z, min_vec.z());
  max_point_.x = std::max(max_point_.x, max_vec.x());
  max_point_.y = std::max(max_point_.y, max_vec.y());
  max_point_.z = std::max(max_point_.z, max_vec.z());
  diameter_ = pcl_utils::ComputeXYDiameter(min_point_, max_point_);

  ++times_of_observed_;
}

const Cluster::PointT &Cluster::centroid() const {
  return clusters_manager_->centroids_cloud()->points.at(id_);
}

Cluster::PointT *Cluster::mutable_centroid() {
  return &clusters_manager_->centroids_cloud()->points.at(id_);
}

const Cluster::Point2d &Cluster::centroid2d() const {
  return clusters_manager_->centroids2d_cloud()->points.at(id_);
}

Cluster::Point2d *Cluster::mutable_centroid2d() {
  return &clusters_manager_->centroids2d_cloud()->points.at(id_);
}

void Cluster::ResetClusterCloudIntensity() {
  const int intensity = std::rand() % 255;
  for (auto &pt : cloud_->points) {
    pt.intensity = intensity;
  }
  this->mutable_centroid()->intensity = intensity;
}

void Cluster::ResetMergeCloudIntensity(const PointCloud::Ptr &cloud_in) {
  const int intensity = cloud_->front().intensity;
  for (auto &pt : cloud_in->points) {
    pt.intensity = intensity;
  }
}

} // namespace long_term_relocalization

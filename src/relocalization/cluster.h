// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/21.

#pragma once

#include <fstream>
#include <memory>

#include <glog/logging.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include "relocalization/serialize.h"
#include "utils/common/common.h"
#include "utils/common/pcl_utils.h"

namespace long_term_relocalization {


struct DistAngle {
  DistAngle() = default;
  DistAngle(double dist, double angle) : dist(dist), angle(angle) {}
  double dist = 0.0;
  double angle = 0.0;
};

struct MiddleOutDescriptor {
  void reset() {
    neighbor_ids.clear();
    edges.clear();
    data.clear();
  }

  int id = 0;
  int num_neighbors = 0;
  std::vector<int> neighbor_ids;
  // N, distance between centorid and neighbors. This vector is sorted.
  std::vector<double> edges;
  // N * N-1
  std::vector<std::vector<DistAngle>> data;
};

class ClustersManager;

class Cluster {
public:
  using Point2d = pcl::PointXY;
  using PointCloud2d = pcl::PointCloud<pcl::PointXY>;
  using PointT = pcl_utils::PointIRL;
  using PointCloud = pcl_utils::PointIRLCloud;
  using Ptr = std::shared_ptr<Cluster>;

  Cluster(const PointCloud::Ptr &cloud, int id, ClustersManager *clusters_manager,
          int times_of_observed = 1);

  virtual ~Cluster() = default;

  void MergeCloud(const PointCloud::Ptr &cloud_in);

  const PointT &centroid() const;
  PointT *mutable_centroid();

  const Point2d &centroid2d() const;
  Point2d *mutable_centroid2d();

  const PointT &min_point() const { return min_point_; }
  const PointT &max_point() const { return max_point_; }

  double height() const { return max_point_.z - min_point_.z; }

  int size() const { return cloud_->size(); }
  double diameter() const { return diameter_; }
  int label() const { return label_; }
  int id() const { return id_; }

  const PointCloud &cloud() const { return *cloud_; }
  PointCloud::Ptr &mutable_cloud() { return cloud_; }

  MiddleOutDescriptor *mutable_middle_out_descriptor() { return &middle_out_descriptor_; }
  const MiddleOutDescriptor &middle_out_descriptor() const { return middle_out_descriptor_; }

  const ClustersManager &clusters_manager() const { return *clusters_manager_; }

  int times_of_observed() const { return times_of_observed_; }
  bool is_observed_enough_times() const { return times_of_observed_ >= 5; }
  int update_observed_times() { ++times_of_observed_; }

  bool is_static_pole() const { return times_of_observed() >= 7 && diameter() < 0.8 && height() > 0.9; }

private:
  void ResetClusterCloudIntensity();
  void ResetMergeCloudIntensity(const PointCloud::Ptr &cloud_in);

  PointCloud::Ptr cloud_;
  PointT min_point_;
  PointT max_point_;
  double diameter_ = 0.0;
  int label_ = 0.0;
  int id_;

  // Filter dynamic clusters. e.g. sometimes, moving person are classified as trunk.
  int times_of_observed_ = 0;

  MiddleOutDescriptor middle_out_descriptor_;

  // The reason we didn't use use `PointT* centroid_` here is for preventing coredump,
  // cause when point cloud in ClustersManager reallocated, the address of centroid_ here become
  // nullptr.
  ClustersManager *clusters_manager_ = nullptr;

  // For performance, we disallow copy, move and assign.
  Cluster(const Cluster &rhs) = delete;
  Cluster(const Cluster &&rhs) = delete;
  Cluster &operator=(const Cluster &rhs) = delete;
};


} // namespace long_term_relocalization

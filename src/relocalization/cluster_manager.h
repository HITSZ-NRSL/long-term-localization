// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/11/2.

#pragma once

#include <mutex>

#include "relocalization/cluster.h"

namespace long_term_relocalization {

class ClustersManager {
  using Point2d = Cluster::Point2d;
  using PointCloud2d = Cluster::PointCloud2d;
  using PointT = Cluster::PointT;
  using PointCloud = Cluster::PointCloud;

public:
  ClustersManager();
  virtual ~ClustersManager() = default;

  void AddNewCluster(const PointCloud::Ptr &cloud_in, int times_of_observed = 1);

  void MergeCloudToOldCluster(const PointCloud::Ptr &cloud_in, int old_cluster_id);

  int num_clusters() const { return global_cluster_id_; }

  const PointCloud2d::Ptr &centroids2d_cloud() const { return centroids2d_cloud_; }
  const PointCloud::Ptr &centroids_cloud() const { return centroids_cloud_; }

  const Cluster::Ptr &at(int id) const;
  Cluster::Ptr &at(int id);

  const std::vector<Cluster::Ptr> &clusters() const { return clusters_; }

  PointCloud::Ptr GetClustersCloud();

  const std::vector<int> &GetLocalClustersIndices() const { return local_clusters_indices_; }

  void SaveClustersToFile(const std::string &filename);

  void LoadClusters(const std::string &filename, double remain_ratio = 1.0);

  void ComputeMiddleOutDescriptors(double search_radius);

  void ComputeMiddleOutDescriptors(double search_radius, const Point2d &cur_pose,
                                   int local_clusters_size);

  // Get static clusters.
  PointCloud::Ptr GetStaticClustersCloud();  // For visualizing.
  PointCloud::Ptr GetStaticCentroidsCloud(); // For visualizing.
  std::pair<PointCloud2d::Ptr, std::vector<int>>
  GetStaticCentroids2dCloudWithIndices(); // For computing middleout descriptors.

  int GetNumStaticClusters();

private:
  int NextId() { return global_cluster_id_++; }
  PointT &NextCentroid();
  Point2d &NextCentroid2d();

  //  Reference Axis: OA, compute angle between OA and OB.
  double ComputeAngleWithReferenceAxis(const Point2d &O, const Point2d &A, const Point2d &B,
                                       double OA, double OB);

  std::vector<Cluster::Ptr> clusters_;
  std::vector<int> local_clusters_indices_;
  std::mutex local_clusters_indices_mutex_;

  // Easy to construct kdtree.
  PointCloud::Ptr centroids_cloud_;
  PointCloud2d::Ptr centroids2d_cloud_;
  int global_cluster_id_ = 0;

  // Disallow copy, move and assign.
  ClustersManager(const ClustersManager &rhs) = delete;
  ClustersManager(const ClustersManager &&rhs) = delete;
  ClustersManager &operator=(const ClustersManager &rhs) = delete;
};

} // namespace long_term_relocalization

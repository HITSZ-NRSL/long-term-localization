// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-10-22.

#pragma once

#include <list>

#include <pcl/kdtree/flann.h>

#include "long_term_relocalization/depth_clustering/depth_clustering.h"
#include "long_term_relocalization/relocalization/cluster_manager.h"
#include "long_term_relocalization/relocalization/keyframe.h"
#include "long_term_relocalization/utils/params_types.h"

namespace long_term_relocalization {

class SemanticClusterMap {
  using Point2d = Cluster::Point2d;
  using PointT = Cluster::PointT;
  using PointCloud = Cluster::PointCloud;
  using KeyFramePtr = KeyFrame<PointT>::Ptr;

public:
  explicit SemanticClusterMap(const SemanticClusterMapParams &params);

  virtual ~SemanticClusterMap() = default;

  void UpdateKeyFrame(const KeyFramePtr &kf);

  const ros::Time &latest_stamp() const { return latest_stamp_; }
  const kindr::minimal::QuatTransformation &latest_pose() const { return latest_pose_; }

  ClustersManager *mutable_cluster_manager() { return &clusters_manager_; }

private:
  std::vector<PointCloud::Ptr> FilterClusters(const std::vector<PointCloud::Ptr> &clusters);

  // Choose trunk and pole.
  bool IsPoleLikeCluster(const PointCloud &cluster);

  void RigisterClusters(const std::vector<PointCloud::Ptr> &clusters_cloud /*in local frame*/,
                        const Eigen::Matrix4d &pose);

  SemanticClusterMapParams params_;

  ClustersManager clusters_manager_;

  ros::Time latest_stamp_;
  kindr::minimal::QuatTransformation latest_pose_;

  pcl::KdTreeFLANN<Cluster::Point2d>::Ptr centroids2d_kdtree_; // Clusters centroids kdtree.
  std::unique_ptr<DepthClustering<PointT>> depth_clustering_;
};

} // namespace long_term_relocalization

// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-10-22.

#include "long_term_relocalization/relocalization/semantic_cluster_map.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

namespace long_term_relocalization {

SemanticClusterMap::SemanticClusterMap(const SemanticClusterMapParams &params) : params_(params) {
  centroids2d_kdtree_ = boost::make_shared<pcl::KdTreeFLANN<Cluster::Point2d>>();
  depth_clustering_ = std::make_unique<DepthClustering<PointT>>(params_.depth_clustering_params);
}

void SemanticClusterMap::UpdateKeyFrame(const SemanticClusterMap::KeyFramePtr &kf) {
  SEGMENT_TIME_BEGIN(SemanticClusterMapUpdateKeyFrame);
  depth_clustering_->Process(*kf->cloud());
  const std::vector<PointCloud::Ptr> clusters = depth_clustering_->ExtractClusters();
  const std::vector<PointCloud::Ptr> filtered_clusters = FilterClusters(clusters);

  const Eigen::Matrix4d pose = kf->pose().getTransformationMatrix();
  RigisterClusters(filtered_clusters, pose);
  latest_stamp_ = kf->stamp();
  latest_pose_ = kf->pose();
  SEGMENT_TIME_END_WHETHER_OVERTIME(SemanticClusterMapUpdateKeyFrame, 120);
}

std::vector<SemanticClusterMap::PointCloud::Ptr> SemanticClusterMap::FilterClusters(
    const std::vector<SemanticClusterMap::PointCloud::Ptr> &clusters) {
  std::vector<PointCloud::Ptr> filtered_clusters;
  filtered_clusters.reserve(clusters.size());

  for (const auto &cluster : clusters) {
    // PointCloud::Ptr cloud(new PointCloud());
    // cloud->reserve(cluster->size());
    // for (const auto &point : cluster->points) {
    //   // Choose static semantic object points.
    //   if (params_.semantic_labels.count(point.label) > 0 && point.z <= params_.max_point_height)
    //   {
    //     cloud->push_back(point);
    //   }
    // }
    if (IsPoleLikeCluster(*cluster)) {
      filtered_clusters.push_back(cluster);
    }
  }

  return filtered_clusters;
}

bool SemanticClusterMap::IsPoleLikeCluster(const SemanticClusterMap::PointCloud &cluster) {
  // 1. filter by size
  if (cluster.size() <= params_.min_cluster_size) {
    return false;
  }

  // 2. filter by label count
  int pole_like_label_count = 0;
  for (const auto &point : cluster) {
    if (params_.semantic_labels.count(point.label) > 0) {
      ++pole_like_label_count;
    }
  }
  if (pole_like_label_count <= 5) {
    return false;
  }

  // 3. filter by geometric conditions
  Eigen::Vector4f min_point, max_point;
  pcl::getMinMax3D(cluster, min_point, max_point);

  const Eigen::Vector4f diff_point = max_point - min_point;
  const double xy_bound =
      std::sqrt(diff_point.x() * diff_point.x() + diff_point.y() * diff_point.y());
  const double height = std::abs(max_point.z() - min_point.z());

  return xy_bound <= params_.max_pole_xy_bound && height > params_.min_pole_height;
}

void SemanticClusterMap::RigisterClusters(
    const std::vector<PointCloud::Ptr> &clusters_cloud /*in local frame*/,
    const Eigen::Matrix4d &pose) {

  if (clusters_manager_.num_clusters() == 0) {
    for (const auto &cloud : clusters_cloud) {
      PointCloud::Ptr cloud_out(new PointCloud());
      pcl::transformPointCloud(*cloud, *cloud_out, pose);
      clusters_manager_.AddNewCluster(cloud_out);
    }
    return;
  }

  // If already initialized, we should build 2d kdtree using registered clusters' centroid and
  // then search each centroid of new clusters in this kdtree.
  centroids2d_kdtree_->setInputCloud(clusters_manager_.centroids2d_cloud());
  for (const auto &cloud : clusters_cloud) {
    // Transform to world frame.
    PointCloud::Ptr cloud_out(new PointCloud());
    pcl::transformPointCloud(*cloud, *cloud_out, pose);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_out, centroid);
    const Cluster::Point2d centroid2d = pcl_utils::Vector4fToPoint2d(centroid);

    std::vector<int> k_indices;
    std::vector<float> k_distances;
    centroids2d_kdtree_->nearestKSearch(centroid2d, 1, k_indices, k_distances);
    CHECK(!k_indices.empty());
    const int old_label = clusters_manager_.at(k_indices[0])->label();
    // TODO(silin): cause the data we labeled is not perfact, so we can not merge by this condition.
    // Merge two cluster. old_label == cloud->front().label
    if (k_distances[0] <= params_.max_register_cluster_distance) {
      // In case of inconsistent cluster cloud caused by multiple registrations.
      // Only update observed times, but do not add cloud.
      if (clusters_manager_.at(k_indices[0])->is_observed_enough_times()) {
        clusters_manager_.at(k_indices[0])->update_observed_times();
      } else {
        clusters_manager_.MergeCloudToOldCluster(cloud_out, k_indices[0]);
      }
    } else {
      // TODO(silin): compute Jaccard Distance, if it is too small, merge them.
      // label = max_num_points_cluster.label
      // Add new cluster.
      clusters_manager_.AddNewCluster(cloud_out);
    }
  }
}

} // namespace long_term_relocalization

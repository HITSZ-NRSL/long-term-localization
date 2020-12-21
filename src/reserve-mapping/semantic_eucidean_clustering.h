// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-1.

#pragma once

#include <unordered_map>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "relocalization/cluster.h"
#include "utils/common/pcl_utils.h"
#include "utils/params/params_types.h"

namespace long_term_relocalization {


// NOTE: Use IncrementalSemanticSegmentation to replace this algorithm.
class SemanticEucideanClustering {
  using PointT = pcl_utils::PointIRL;
  using PointCloud = pcl_utils::PointIRLCloud;

public:
  explicit SemanticEucideanClustering(const params::SemanticEucideanClusteringParams &params);

  virtual ~SemanticEucideanClustering() = default;

  // Pipline:
  // 1. segment all points into semantic_seeds_clusters by semantic label
  // 2. use eucidean clustering algorithm to segment all semantic_seeds_clusters, only keep size >
  // threshold
  // 3. use points in semantic_seeds_clusters and input cloud, run SemanticEucideanSegmentation to
  // get the final semantic clusters
  void Process(const PointCloud &cloud_in);

  void ExtractSemanticRawClusters(const PointCloud &cloud_in);

  void MergeSemanticSeedsClusters();

  PointCloud::Ptr GetSemanticClusters() {
    PointCloud::Ptr cloud(new PointCloud());
    for (const auto &item : seed_points_) {
      *cloud += *item.second;
    }

    return cloud;
  }

private:
  std::vector<pcl::PointIndices> EucideanSegmentation(
      const PointCloud::Ptr &cloud,
      params::SemanticEucideanClusteringParams::ObjectEucideanParams &obj_eucidean_params);

  std::vector<pcl::PointIndices> SemanticEucideanSegmentation(const PointCloud::Ptr &cloud,
                                                              const std::vector<int> &seed_indices);

  bool IsPoleLikeCluster(const PointCloud &cluster);

  void ResetParams();

  params::SemanticEucideanClusteringParams params_;
  std::unordered_map<int /*label*/, PointCloud::Ptr> seed_points_;
  std::unordered_map<int /*label*/, std::vector<Cluster::Ptr>> raw_clusters_;
};


} // namespace long_term_relocalization

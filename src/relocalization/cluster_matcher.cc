// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/11/2.

#include "relocalization/cluster_matcher.h"

namespace long_term_relocalization {

//============================================================================
// MiddleOutDescriptor Matching Algorithm Implementation
// TODO(silin): REFACTOR in the future.
//============================================================================
double ClusterMatcher::ComputeDistanceBetweenDistAngles(const DistAngle &lhs,
                                                        const DistAngle &rhs) {
  return std::sqrt(lhs.dist * lhs.dist + rhs.dist * rhs.dist -
                   2 * lhs.dist * rhs.dist * std::cos(std::abs(lhs.angle - rhs.angle)));
}

int ClusterMatcher::ComputeDistanceBetweenNeighborEdges(const std::vector<DistAngle> &source,
                                                        const std::vector<DistAngle> &target,
                                                        double *distance) {
  CHECK(distance != nullptr);
  int dist_angle_matches_count = 0;
  double sum_diff_dist_angle = 0.0;

  // FOR DEBUG.
  std::unordered_map<int, std::vector<std::pair<int /*tar index*/, double /*min_diff_dist_angle*/>>>
      matched_edges;
  for (int src_ind = 0; src_ind < source.size(); ++src_ind) {
    double min_diff_dist_angle = std::numeric_limits<double>::max();
    int matched_target_ind = -1;

    for (int tar_ind = 0; tar_ind < target.size(); ++tar_ind) {
      if (std::abs(target[tar_ind].dist - source[src_ind].dist) > params_.max_diff_dist ||
          std::abs(target[tar_ind].angle - source[src_ind].angle) > params_.max_diff_angle) {
        continue;
      }
      const double diff_dist_angle =
          ComputeDistanceBetweenDistAngles(target[tar_ind], source[src_ind]);
      if (diff_dist_angle < min_diff_dist_angle) {
        min_diff_dist_angle = diff_dist_angle;
        matched_target_ind = tar_ind;
      }
    }

    if (min_diff_dist_angle < params_.max_diff_dist_angle) {
      // LOG(INFO) << "min_diff_dist_angle: " << min_diff_dist_angle;
      ++dist_angle_matches_count;
      sum_diff_dist_angle += min_diff_dist_angle;
      matched_edges[src_ind].emplace_back(matched_target_ind, min_diff_dist_angle);
    }
  }

  // LOG(INFO) << "****************************************";
  // for (const auto &item : matched_edges) {
  //   LOG(INFO) << "src ind: " << item.first;
  //   LOG(INFO) << "tar ind\t | min diff dist angle";
  //   for (const auto &tar : item.second) {
  //     LOG(INFO) << tar.first << "\t" << tar.second;
  //   }
  // }

  *distance = sum_diff_dist_angle * (1.0 / dist_angle_matches_count - 1.0 / source.size());
  return dist_angle_matches_count;
}

bool ClusterMatcher::AreTwoClustersMatched(const Cluster::Ptr &source_cluster,
                                           const Cluster::Ptr &target_cluster) {
  const MiddleOutDescriptor &tar_descriptor = target_cluster->middle_out_descriptor();
  const MiddleOutDescriptor &src_descriptor = source_cluster->middle_out_descriptor();
  // 0. Same label.
  // if (target_cluster->label() != source_cluster->label()) {
  //   return false;
  // }

  // 1. Difference number of neighbors.
  // const int diff_num_neighbors =
  //     std::abs(tar_descriptor.num_neighbors - src_descriptor.num_neighbors);

  // if (diff_num_neighbors > params_.max_diff_num_neighbors) {
  //   return false;
  // }

  // 2. Find edge pairs.
  EdgePairWiseMatches edge_matches;
  int neighbors_cluster_matches_count = 0;
  double sum_dist_of_neighbor_cluster_match = 0.0;
  for (int src_neighbor_ind = 0; src_neighbor_ind < src_descriptor.num_neighbors;
       ++src_neighbor_ind) {
    // Sub-descriptor
    const std::vector<DistAngle> &src_neighbor_data = src_descriptor.data[src_neighbor_ind];
    const std::vector<int> edge_candidate_indices = FindClosestValues(
        tar_descriptor.edges, src_descriptor.edges[src_neighbor_ind], params_.num_closest_edges);

    //------------------------------
    // FOR DEBUG
    EdgePairWiseMatch edge_match;
    const int src_neighbor_cluster_id = src_descriptor.neighbor_ids[src_neighbor_ind];
    edge_match.src_edge =
        PairwiseMatch(source_cluster->id(), src_neighbor_cluster_id, source_cluster->centroid(),
                      source_cluster->clusters_manager().at(src_neighbor_cluster_id)->centroid());
    //------------------------------

    double min_dist_of_neighbor_cluster_match = std::numeric_limits<double>::max();
    int num_edge_matches = 0;
    for (const int tar_neighbot_ind : edge_candidate_indices) {
      const double diff_edge =
          std::abs(tar_descriptor.edges[tar_neighbot_ind] - src_descriptor.edges[src_neighbor_ind]);
      if (diff_edge > params_.max_diff_dist) {
        continue;
      }
      const std::vector<DistAngle> &tar_neighbor_data = tar_descriptor.data[tar_neighbot_ind];
      double distance = 0;
      const int dist_angle_matches_count =
          ComputeDistanceBetweenNeighborEdges(src_neighbor_data, tar_neighbor_data, &distance);
      if (dist_angle_matches_count > params_.min_num_edges_matches) {
        //------------------------------
        // FOR DEBUG
        // LOG(INFO) << "dist_angle_matches_count :\t" << dist_angle_matches_count << " distance:\t"
        //           << distance;
        const int tar_neighbor_cluster_id = tar_descriptor.neighbor_ids[tar_neighbot_ind];
        edge_match.tar_edge = PairwiseMatch(
            target_cluster->id(), tar_neighbor_cluster_id, target_cluster->centroid(),
            target_cluster->clusters_manager().at(tar_neighbor_cluster_id)->centroid());
        edge_matches.emplace_back(edge_match);

        min_dist_of_neighbor_cluster_match = std::min(min_dist_of_neighbor_cluster_match, distance);
        //------------------------------
      }
    } // Traverse tar neighbors end.

    if (min_dist_of_neighbor_cluster_match < params_.max_dist_of_neighbor_cluster_match) {
      // LOG(INFO) << "min_dist_of_neighbor_cluster_match: " << min_dist_of_neighbor_cluster_match;
      ++neighbors_cluster_matches_count;
      sum_dist_of_neighbor_cluster_match += min_dist_of_neighbor_cluster_match;
    }
  } // Traverse src neighbors end.

  double final_distance =
      sum_dist_of_neighbor_cluster_match *
      (1.0 / neighbors_cluster_matches_count - 1.0 / src_descriptor.num_neighbors);
  if (neighbors_cluster_matches_count >= 5) {
    // LOG(INFO) << "neighbors_cluster_matches_count: " << neighbors_cluster_matches_count;
    // LOG(INFO) << "final distance: " << final_distance;

    return true;
  }
  return false;
}

} // namespace long_term_relocalization

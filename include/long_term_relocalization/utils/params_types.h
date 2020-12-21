// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/20.

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace long_term_relocalization {

struct GroundPlaneFitterParams {
  double gpf_sensor_height = 1.73;
  // number of iterations
  int gpf_num_iter;
  // number of points used to estimate the lowest point representative(LPR)
  // double of senser model???
  int gpf_num_lpr;
  double gpf_th_lprs;
  // threshold for points to be considered initial seeds
  double gpf_th_seeds;
  // ground points threshold distance from the plane
  //   <== large to guarantee safe removal
  double gpf_th_gnds;
};

struct DepthClusteringParams {
  int width = 0;
  int height = 0;

  double max_ground_diff_angle = 0.0; // rad
  double max_ground_seed_angle = 0.0; // rad
  double max_ground_diff_z = 0.0;

  double max_cluster_distance = 0;
  double min_cluster_angle = 0; // rad
  int min_cluster_size = 0;
  double min_good_cluster_diff_z = 0.0;

  std::vector<double> vert_angles; // degree
  std::vector<double> hori_angles; // degree

  int ground_semantic_label = 0;
  int sidewalk_semantic_label = 0;
  GroundPlaneFitterParams ground_plane_fitter_params;
};

struct GeometricConsistencyParams {
  // Type of recognizer.
  std::string recognizer_type;
  // Higher resolutions lead to higher tolerances.
  double resolution = 0.2;
  // Minimum number of matches necessary to consider cluster.
  int min_cluster_size = 10;
  // Maximum consistency distance between two matches in order for them to be cached as candidates.
  // Used in the incremental recognizer only.
  float max_consistency_distance_for_caching = 10.0f;
  float local_map_radius;
};

struct SemanticClusterMapParams {
  std::unordered_set<int> semantic_labels;

  // cluster filtering conditions.
  double max_point_height;
  int min_cluster_size = 0;
  double min_pole_height = 0.0;
  double max_pole_xy_bound = 0.0;

  double max_register_cluster_distance = 0.0;

  DepthClusteringParams depth_clustering_params;
};

struct ClusterMatcherParams {
  int max_diff_num_neighbors = 0.0;
  int num_closest_edges = 0.0;
  double max_diff_dist = 0.0;
  double max_diff_angle = 0.0 /*radian*/;
  double max_diff_dist_angle = 0.0 /*m*/;
  int min_num_edges_matches = 0;
  double max_dist_of_neighbor_cluster_match = 0.0;
};

struct RelocalizationParams {
  std::string mode;
  bool evalute;
  double remain_ratio;

  double search_radius = 0.0;
  int local_cluster_size = 0;

  double downsample_leaf_size = 0.0;   // For constructing global map.
  int max_num_frames_in_local_map = 0; // Construct local map.
  int min_num_frames_in_local_map = 0;
  double distance_to_lower_target_cloud_for_viz_m = 0.0;

  SemanticClusterMapParams semantic_cluster_map_params;
  ClusterMatcherParams cluster_matcher_params;
  GeometricConsistencyParams geometric_consistency_params;
};

} // namespace long_term_relocalization

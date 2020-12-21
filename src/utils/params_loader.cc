// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/20.

#include "long_term_relocalization/utils/params_loader.h"

#include "common/file/csv_reader.h"
#include "common/math/math.h"
#include "long_term_relocalization/utils/common.h"

namespace long_term_relocalization {

ParamsLoader::ParamsLoader(const std::string &config_file, const std::string &angle_file) {
  file::YamlReader yaml_reader(config_file);
  LoadGroundPlaneFitterParams(file::YamlReader(yaml_reader.ReadSubNode("ground_plane_fitter")));
  LoadDepthClusteringParams(angle_file,
                            file::YamlReader(yaml_reader.ReadSubNode("depth_clustering")));

  LoadSemanticClusterMapParams(file::YamlReader(yaml_reader.ReadSubNode("semantic_cluster_map")));
  LoadClusterMatcherParams(file::YamlReader(yaml_reader.ReadSubNode("cluster_matcher")));
  LoadGeometricConsistencyParams(
      file::YamlReader(yaml_reader.ReadSubNode("geometric_consistency")));

  LoadRelocalizationParams(file::YamlReader(yaml_reader.ReadSubNode("relocalization")));
}
void ParamsLoader::LoadGroundPlaneFitterParams(file::YamlReader subnode_reader) {
  ground_plane_fitter_params_.gpf_sensor_height =
      subnode_reader.ReadValue<double>("gpf_sensor_height");
  ground_plane_fitter_params_.gpf_num_iter = subnode_reader.ReadValue<int>("gpf_num_iter");
  ground_plane_fitter_params_.gpf_num_lpr = subnode_reader.ReadValue<int>("gpf_num_lpr");
  ground_plane_fitter_params_.gpf_th_lprs = subnode_reader.ReadValue<double>("gpf_th_lprs");
  ground_plane_fitter_params_.gpf_th_seeds = subnode_reader.ReadValue<double>("gpf_th_seeds");
  ground_plane_fitter_params_.gpf_th_gnds = subnode_reader.ReadValue<double>("gpf_th_gnds");

  LOG(INFO) << "---------------ground_plane_fitter_params---------------";
  LOG(INFO) << "gpf_num_iter: " << ground_plane_fitter_params_.gpf_num_iter;
  LOG(INFO) << "gpf_num_lpr: " << ground_plane_fitter_params_.gpf_num_lpr;
  LOG(INFO) << "gpf_th_lprs: " << ground_plane_fitter_params_.gpf_th_lprs;
  LOG(INFO) << "gpf_th_seeds: " << ground_plane_fitter_params_.gpf_th_seeds;
  LOG(INFO) << "gpf_th_gnds: " << ground_plane_fitter_params_.gpf_th_gnds;
  LOG(INFO) << "--------------------------------------------------------";
}

void ParamsLoader::LoadDepthClusteringParams(const std::string &angle_file,
                                             file::YamlReader subnode_reader) {
  depth_clustering_params_.width = subnode_reader.ReadValue<int>("width");
  depth_clustering_params_.height = subnode_reader.ReadValue<int>("height");
  depth_clustering_params_.max_ground_seed_angle =
      math::DegToRad(subnode_reader.ReadValue<double>("max_ground_seed_angle"));
  depth_clustering_params_.max_ground_diff_angle =
      math::DegToRad(subnode_reader.ReadValue<double>("max_ground_diff_angle"));
  depth_clustering_params_.max_ground_diff_z =
      subnode_reader.ReadValue<double>("max_ground_diff_z");
  depth_clustering_params_.max_cluster_distance =
      subnode_reader.ReadValue<double>("max_cluster_distance");
  depth_clustering_params_.min_cluster_angle =
      math::DegToRad(subnode_reader.ReadValue<double>("min_cluster_angle"));
  depth_clustering_params_.min_cluster_size = subnode_reader.ReadValue<int>("min_cluster_size");
  depth_clustering_params_.min_good_cluster_diff_z =
      subnode_reader.ReadValue<double>("min_good_cluster_diff_z");

  depth_clustering_params_.ground_semantic_label =
      subnode_reader.ReadValue<int>("ground_semantic_label");
  depth_clustering_params_.sidewalk_semantic_label =
      subnode_reader.ReadValue<int>("sidewalk_semantic_label");

  std::tie(depth_clustering_params_.vert_angles, depth_clustering_params_.hori_angles) =
      LoadRs80CalibrationFile(angle_file);

  depth_clustering_params_.ground_plane_fitter_params = this->get_ground_plane_fitter_params();

  LOG(INFO) << "---------------depth_clustering_params---------------";
  LOG(INFO) << "width: " << depth_clustering_params_.width;
  LOG(INFO) << "height: " << depth_clustering_params_.height;
  LOG(INFO) << "max_ground_seed_angle: "
            << math::RadToDeg(depth_clustering_params_.max_ground_seed_angle) << " degree.";
  LOG(INFO) << "max_ground_diff_angle: "
            << math::RadToDeg(depth_clustering_params_.max_ground_diff_angle) << " degree.";
  LOG(INFO) << "max_ground_diff_z: " << depth_clustering_params_.max_ground_diff_z;
  LOG(INFO) << "min_cluster_angle: " << math::RadToDeg(depth_clustering_params_.min_cluster_angle)
            << " degree.";
  LOG(INFO) << "max_cluster_distance: " << depth_clustering_params_.max_cluster_distance;
  LOG(INFO) << "min_cluster_size: " << depth_clustering_params_.min_cluster_size;
  LOG(INFO) << "min_good_cluster_diff_z: " << depth_clustering_params_.min_good_cluster_diff_z;
  LOG(INFO) << "ground_semantic_label: " << depth_clustering_params_.ground_semantic_label;
  LOG(INFO) << "sidewalk_semantic_label: " << depth_clustering_params_.sidewalk_semantic_label;
  //   TraverseLogOut(depth_clustering_params_.vert_angles, true);
  //   TraverseLogOut(depth_clustering_params_.hori_angles, true);
  LOG(INFO) << "-----------------------------------------------------";
}

std::vector<double> ParamsLoader::LoadVelodyne32LaserAngles(const std::string &angle_file) {
  file::CsvReader csv_reader(angle_file, ',');

  std::vector<double> angles;
  std::vector<std::string> strs;
  while (csv_reader.GetLine(&strs)) {
    angles.emplace_back(math::RadToDeg(std::stod(strs.back())));
  }
  CHECK_EQ(angles.size(), 32);
  std::sort(angles.begin(), angles.end(), std::greater<double>());

  return angles; // degree
}

std::tuple<std::vector<double>, std::vector<double>>
ParamsLoader::LoadRs80CalibrationFile(const std::string &angle_file) {
  std::vector<double> vert_angles(80); // degree
  std::vector<double> hori_angles(80); // degree

  std::string line_str;
  std::ifstream fd_angle(angle_file.c_str(), std::ios::in);
  CHECK(fd_angle.is_open());
  unsigned int row_index = 0;
  while (std::getline(fd_angle, line_str)) {
    std::stringstream ss(line_str);
    std::string str;
    std::vector<std::string> vect_str;
    while (std::getline(ss, str, ',')) {
      vect_str.emplace_back(str);
    }
    try {
      vert_angles[row_index] = std::stod(vect_str.at(0));
      hori_angles[row_index] = std::stod(vect_str.at(1));
    } catch (...) {
      LOG(ERROR) << "Wrong calibration file format! Please check your angle.csv file!";
      break;
    }
    row_index++;
  }
  fd_angle.close();

  return std::make_tuple(vert_angles, hori_angles);
}

void ParamsLoader::LoadSemanticClusterMapParams(file::YamlReader subnode_reader) {
  std::vector<int> semantic_labels = subnode_reader.ReadValue<std::vector<int>>("semantic_labels");
  semantic_cluster_map_params_.semantic_labels.insert(semantic_labels.begin(),
                                                      semantic_labels.end());
  semantic_cluster_map_params_.max_point_height =
      subnode_reader.ReadValue<double>("max_point_height");
  semantic_cluster_map_params_.min_cluster_size = subnode_reader.ReadValue<int>("min_cluster_size");
  semantic_cluster_map_params_.min_pole_height =
      subnode_reader.ReadValue<double>("min_pole_height");
  semantic_cluster_map_params_.max_pole_xy_bound =
      subnode_reader.ReadValue<double>("max_pole_xy_bound");
  semantic_cluster_map_params_.max_register_cluster_distance =
      subnode_reader.ReadValue<double>("max_register_cluster_distance");

  semantic_cluster_map_params_.depth_clustering_params = this->get_depth_clustering_params();

  CHECK_EQ(semantic_cluster_map_params_.semantic_labels.size(), 2);
  LOG(INFO) << "---------------semantic_cluster_map_params---------------";
  LOG(INFO) << "semantic_labels: ";
  common::TraverseLogOut(semantic_labels);
  LOG(INFO) << "max_point_height: " << semantic_cluster_map_params_.max_point_height;
  LOG(INFO) << "min_cluster_size: " << semantic_cluster_map_params_.min_cluster_size;
  LOG(INFO) << "min_pole_height: " << semantic_cluster_map_params_.min_pole_height;
  LOG(INFO) << "max_pole_xy_bound: " << semantic_cluster_map_params_.max_pole_xy_bound;
  LOG(INFO) << "max_register_cluster_distance: "
            << semantic_cluster_map_params_.max_register_cluster_distance;
  LOG(INFO) << "--------------------------------------------------------";
}

void ParamsLoader::LoadClusterMatcherParams(file::YamlReader subnode_reader) {
  cluster_matcher_params_.max_diff_num_neighbors =
      subnode_reader.ReadValue<int>("max_diff_num_neighbors");
  cluster_matcher_params_.num_closest_edges = subnode_reader.ReadValue<int>("num_closest_edges");
  cluster_matcher_params_.max_diff_dist = subnode_reader.ReadValue<double>("max_diff_dist");
  cluster_matcher_params_.max_diff_angle =
      math::DegToRad(subnode_reader.ReadValue<double>("max_diff_angle"));
  cluster_matcher_params_.max_diff_dist_angle =
      subnode_reader.ReadValue<double>("max_diff_dist_angle");
  cluster_matcher_params_.min_num_edges_matches =
      subnode_reader.ReadValue<int>("min_num_edges_matches");
  cluster_matcher_params_.max_dist_of_neighbor_cluster_match =
      subnode_reader.ReadValue<double>("max_dist_of_neighbor_cluster_match");

  LOG(INFO) << "---------------cluster_matcher_params---------------";
  LOG(INFO) << "max_diff_num_neighbors: " << cluster_matcher_params_.max_diff_num_neighbors;
  LOG(INFO) << "num_closest_edges: " << cluster_matcher_params_.num_closest_edges;
  LOG(INFO) << "max_diff_dist: " << cluster_matcher_params_.max_diff_dist;
  LOG(INFO) << "max_diff_angle: " << math::RadToDeg(cluster_matcher_params_.max_diff_angle);
  LOG(INFO) << "max_diff_dist_angle: " << cluster_matcher_params_.max_diff_dist_angle;
  LOG(INFO) << "min_num_edges_matches: " << cluster_matcher_params_.min_num_edges_matches;
  LOG(INFO) << "max_dist_of_neighbor_cluster_match: "
            << cluster_matcher_params_.max_dist_of_neighbor_cluster_match;
  LOG(INFO) << "-------------------------------------------------------------";
}

void ParamsLoader::LoadGeometricConsistencyParams(file::YamlReader subnode_reader) {
  geometric_consistency_params_.recognizer_type =
      subnode_reader.ReadValue<std::string>("recognizer_type");
  geometric_consistency_params_.resolution = subnode_reader.ReadValue<double>("resolution");
  geometric_consistency_params_.min_cluster_size =
      subnode_reader.ReadValue<int>("min_cluster_size");
  geometric_consistency_params_.max_consistency_distance_for_caching =
      subnode_reader.ReadValue<double>("max_consistency_distance_for_caching");
  geometric_consistency_params_.local_map_radius =
      subnode_reader.ReadValue<double>("local_map_radius");

  LOG(INFO) << "---------------geometric_consistency_params---------------";
  LOG(INFO) << "recognizer_type: " << geometric_consistency_params_.recognizer_type;
  LOG(INFO) << "resolution: " << geometric_consistency_params_.resolution;
  LOG(INFO) << "min_cluster_size: " << geometric_consistency_params_.min_cluster_size;
  LOG(INFO) << "max_consistency_distance_for_caching: "
            << geometric_consistency_params_.max_consistency_distance_for_caching;
  LOG(INFO) << "local_map_radius: " << geometric_consistency_params_.local_map_radius;
  LOG(INFO) << "----------------------------------------------------------";
}

void ParamsLoader::LoadRelocalizationParams(file::YamlReader subnode_reader) {
  relocalization_params_.mode = subnode_reader.ReadValue<std::string>("mode");
  relocalization_params_.evalute = subnode_reader.ReadValue<bool>("evalute");
  relocalization_params_.remain_ratio = subnode_reader.ReadValue<double>("remain_ratio");

  relocalization_params_.search_radius = subnode_reader.ReadValue<double>("search_radius");
  relocalization_params_.local_cluster_size = subnode_reader.ReadValue<int>("local_cluster_size");

  relocalization_params_.downsample_leaf_size =
      subnode_reader.ReadValue<double>("downsample_leaf_size");
  relocalization_params_.max_num_frames_in_local_map =
      subnode_reader.ReadValue<int>("max_num_frames_in_local_map");
  relocalization_params_.min_num_frames_in_local_map =
      subnode_reader.ReadValue<int>("min_num_frames_in_local_map");

  relocalization_params_.distance_to_lower_target_cloud_for_viz_m =
      subnode_reader.ReadValue<double>("distance_to_lower_target_cloud_for_viz_m");

  relocalization_params_.semantic_cluster_map_params = this->get_semantic_cluster_map_params();
  relocalization_params_.cluster_matcher_params = this->get_cluster_matcher_params();
  relocalization_params_.geometric_consistency_params = this->get_geometric_consistency_params();

  CHECK(relocalization_params_.mode == "localization" ||
        relocalization_params_.mode == "relocalization");
  CHECK_GT(relocalization_params_.downsample_leaf_size, 0);
  CHECK_GT(relocalization_params_.max_num_frames_in_local_map, 0);

  LOG(INFO) << "---------------semantic_eucidean_clustering_params---------------";
  LOG(INFO) << "mode: " << relocalization_params_.mode;
  LOG(INFO) << "evalute: " << relocalization_params_.evalute;
  LOG(INFO) << "remain_ratio: " << relocalization_params_.remain_ratio;

  LOG(INFO) << "search_radius: " << relocalization_params_.search_radius;
  LOG(INFO) << "local_cluster_size: " << relocalization_params_.local_cluster_size;

  LOG(INFO) << "downsample_leaf_size: " << relocalization_params_.downsample_leaf_size;
  LOG(INFO) << "max_num_frames_in_local_map: "
            << relocalization_params_.max_num_frames_in_local_map;
  LOG(INFO) << "min_num_frames_in_local_map: "
            << relocalization_params_.min_num_frames_in_local_map;

  LOG(INFO) << "distance_to_lower_target_cloud_for_viz_m: "
            << relocalization_params_.distance_to_lower_target_cloud_for_viz_m;
  LOG(INFO) << "-----------------------------------------------------------------";
}

} // namespace long_term_relocalization

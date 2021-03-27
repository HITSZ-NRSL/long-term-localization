// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/20.

#pragma once

#include <string>

#include <glog/logging.h>

#include "common/file/yaml_reader.h"
#include "long_term_relocalization/utils/params_types.h"

namespace long_term_relocalization {

class ParamsLoader {
public:
  ParamsLoader(const std::string &config_file, const std::string &angle_file);

  virtual ~ParamsLoader() = default;

  void LoadGroundPlaneFitterParams(file::YamlReader subnode_reader);

  void LoadDepthClusteringParams(const std::string &angle_file, file::YamlReader subnode_reader);

  void LoadSemanticClusterMapParams(file::YamlReader subnode_reader);

  void LoadClusterMatcherParams(file::YamlReader subnode_reader);

  void LoadRelocalizationParams(file::YamlReader subnode_reader);

  void LoadGeometricConsistencyParams(file::YamlReader subnode_reader);

  const GroundPlaneFitterParams &get_ground_plane_fitter_params() const {
    return ground_plane_fitter_params_;
  }

  const DepthClusteringParams &get_depth_clustering_params() const {
    return depth_clustering_params_;
  }

  const SemanticClusterMapParams &get_semantic_cluster_map_params() const {
    return semantic_cluster_map_params_;
  }

  const ClusterMatcherParams &get_cluster_matcher_params() const { return cluster_matcher_params_; }

  const RelocalizationParams &get_relocalization_params() const { return relocalization_params_; }

  const GeometricConsistencyParams &get_geometric_consistency_params() const {
    return geometric_consistency_params_;
  }

private:
  std::vector<double> LoadVelodyne32LaserAngles(const std::string &angle_file);

  std::tuple<std::vector<double>, std::vector<double>>
  LoadRs80CalibrationFile(const std::string &angle_file);

  GroundPlaneFitterParams ground_plane_fitter_params_;
  DepthClusteringParams depth_clustering_params_;

  SemanticClusterMapParams semantic_cluster_map_params_;
  ClusterMatcherParams cluster_matcher_params_;
  RelocalizationParams relocalization_params_;
  GeometricConsistencyParams geometric_consistency_params_;
};

} // namespace long_term_relocalization

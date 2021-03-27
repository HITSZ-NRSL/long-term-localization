/**
 * @Copyright (c) 2020. All rights reserved
 * @Author supengcc@163.com(Alex Su)
 * @create 2020/9/21
 */
#pragma once

#include <memory>

#include <Eigen/Dense>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

#include "mapping/keyframe.h"
#include "mapping/local_map.h"
#include "common/file/file_path.h"
#include "common/pcl_utils/pcl_types.h"
#include "kindr/minimal/quat-transformation.h"

#include "semantic_octomap/semantic_octree.h"

namespace long_term_relocalization {
namespace mapping {
class SemanticOctoMapGenerator {
  using PointCloud = pcl_utils::PointRGBLCloud;
  using PointCloudPtr = pcl_utils::PointRGBLCloud::Ptr;

public:
  typedef std::unique_ptr<SemanticOctoMapGenerator> UniqPtr;
  explicit SemanticOctoMapGenerator(float octree_size);
  void AddKeyFrame(const ros::Time &stamp, const kindr::minimal::QuatTransformation &pose,
                   const PointCloud &cloud_in);
  const octomap::SemanticOcTree &GenerateGlobalSemanticOctoMap();
  void UpdateSemanticGlobalCloudMap();
  const PointCloud &GenerateGlobalSemanticPointCloudMap();
  int num_semantic_keyframes() const { return semantic_keyframes_.size(); }
  const ros::Time &get_latest_time() {
    CHECK(!semantic_keyframes_.empty());
    return semantic_keyframes_.back()->stamp();
  }

private:
  std::vector<KeyFrame<pcl_utils::PointRGBL>::Ptr> semantic_keyframes_;
  PointCloudPtr semantic_global_point_cloud_map_;
  boost::shared_ptr<pcl::VoxelGrid<pcl_utils::PointRGBL>> voxel_grid_;
  octomap::SemanticOcTree::Ptr semantic_global_octo_map_;
};
} // namespace mapping
} // namespace long_term_relocalization
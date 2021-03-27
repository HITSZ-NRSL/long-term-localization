/**
 * @Copyright (c) 2020. All rights reserved
 * @Author supengcc@163.com(Alex Su)
 * @create 2020/9/21
 */
#include "semantic_octomap_generator.h"

namespace long_term_relocalization {
namespace mapping {
SemanticOctoMapGenerator::SemanticOctoMapGenerator(float octree_size) {
  LOG(INFO) << "------> SemanticOctoMapGenerator Generator Start! <------";
  LOG(INFO) << "octree resolution: " << octree_size;
  semantic_global_point_cloud_map_.reset(new PointCloud);
  voxel_grid_ = boost::make_shared<pcl::VoxelGrid<pcl_utils::PointRGBL>>();
  voxel_grid_->setLeafSize(octree_size, octree_size, octree_size);
  semantic_global_octo_map_.reset(new octomap::SemanticOcTree(octree_size));
  LOG(INFO) << "<------ SemanticOctoMapGenerator Generator Done ------>";
}

void SemanticOctoMapGenerator::AddKeyFrame(const ros::Time &stamp,
                                           const kindr::minimal::QuatTransformation &pose,
                                           const SemanticOctoMapGenerator::PointCloud &cloud_in) {
  KeyFrame<pcl_utils::PointRGBL>::Ptr semantic_keyframe =
      boost::make_shared<KeyFrame<pcl_utils::PointRGBL>>(stamp, pose, cloud_in);
  semantic_keyframes_.push_back(semantic_keyframe);
}

const octomap::SemanticOcTree &SemanticOctoMapGenerator::GenerateGlobalSemanticOctoMap() {
  CHECK(!semantic_keyframes_.empty());
  for (const auto &p : *semantic_global_point_cloud_map_) {
    semantic_global_octo_map_->updateNode(octomap::point3d(p.x, p.y, p.z), true);
    semantic_global_octo_map_->intergrateNodeLabel(p.x, p.y, p.z, p.label);
  }
  semantic_global_octo_map_->updateInnerOccupancy();
  return *semantic_global_octo_map_;
}

const SemanticOctoMapGenerator::PointCloud &
SemanticOctoMapGenerator::GenerateGlobalSemanticPointCloudMap() {
  CHECK(!semantic_keyframes_.empty());
  LOG(INFO) << "global semantic map points -:" << semantic_global_point_cloud_map_->size();
  voxel_grid_->setInputCloud(semantic_global_point_cloud_map_);
  voxel_grid_->filter(*semantic_global_point_cloud_map_);
  LOG(INFO) << "global semantic map points ↓:" << semantic_global_point_cloud_map_->size();
  return *semantic_global_point_cloud_map_;
}
void SemanticOctoMapGenerator::UpdateSemanticGlobalCloudMap() {
  semantic_global_point_cloud_map_->clear();
  PointCloud cloud_out;
  for (const auto &skf : semantic_keyframes_) {
    pcl::transformPointCloud(*skf->cloud(), cloud_out, transform::Rigid3dToMatrix4d(skf->pose()));
    *semantic_global_point_cloud_map_ += cloud_out;
  }
}

} // namespace mapping
} // namespace long_term_relocalization
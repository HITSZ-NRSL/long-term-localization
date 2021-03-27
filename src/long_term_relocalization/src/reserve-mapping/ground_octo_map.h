// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-19.

#pragma once

#include <memory>

#include <pcl/common/transforms.h>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include "common/pcl_utils/pcl_utils.h"
#include "kindr/minimal/quat-transformation.h"
#include "long_term_relocalization/utils/params_types.h"

namespace long_term_relocalization {
namespace mapping {

class GroundOctoMap {
public:
  explicit GroundOctoMap(const GroundOctoMapParams &params) : params_(params) {
    octree_ = std::make_unique<octomap::OcTree>(params_.octomap_resolution);
  }
  virtual ~GroundOctoMap() = default;

  // road label: 0
  // sidewalk label: 1
  void UpdatePointCloud(const pcl_utils::PointIRLCloud &semantic_cloud,
                        const kindr::minimal::QuatTransformation &transform) {
    pcl_utils::PointICloud::Ptr cloud_out(new pcl_utils::PointICloud());
    common::ExtractSemanticPoints(semantic_cloud, params_.ground_semantic_label, cloud_out);
    pcl::transformPointCloud(*cloud_out, *cloud_out, transform.getTransformationMatrix());
    for (const auto &pt : cloud_out->points) {
      const octomap::point3d pt3d(pt.x, pt.y, pt.z);
      octree_->updateNode(pt3d, true);
    }
  }

  const octomap::OcTree &get_octo_map() const { return *octree_; }

private:
  GroundOctoMapParams params_;
  std::unique_ptr<octomap::OcTree> octree_;
};

} // namespace mapping
} // namespace long_term_relocalization

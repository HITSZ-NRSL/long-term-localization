// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/9.

#pragma once

#include <pcl/common/transforms.h>

#include "utils/common/pcl_types.h"
#include "utils/common/time.h"
#include "utils/transform/rigid_transfrom.h"

namespace long_term_relocalization {

template <typename PointT> class KeyFrame {
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

public:
  using Ptr = boost::shared_ptr<KeyFrame<PointT>>;

  KeyFrame(const common::Time &stamp, const transform::Rigid3d &pose, const PointCloud &cloud)
      : stamp_(stamp), pose_(pose), cloud_(new PointCloud(cloud)),
        transformed_cloud_(new PointCloud()), id_(global_keyframe_id_++) {
    pcl::transformPointCloud(*cloud_, *transformed_cloud_, transform::Rigid3dToMatrix4d(pose));
  }

  virtual ~KeyFrame() = default;

  int id() const { return id_; }
  const common::Time &stamp() const { return stamp_; }
  const transform::Rigid3d &pose() const { return pose_; }
  const PointCloudPtr &cloud() const { return cloud_; }
  const PointCloudPtr &transformed_cloud() const { return transformed_cloud_; }

private:
  common::Time stamp_;
  transform::Rigid3d pose_;
  PointCloudPtr cloud_;
  PointCloudPtr transformed_cloud_;
  int id_ = 0;

  static int global_keyframe_id_;
};

template <typename PointT> int KeyFrame<PointT>::global_keyframe_id_ = 0;

} // namespace long_term_relocalization

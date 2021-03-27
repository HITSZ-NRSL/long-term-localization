// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-22.

#include <glog/logging.h>

#include "common/pcl_utils/pcl_utils.h"

namespace pcl_utils {

void ExtractSemanticPointsIndices(const pcl_utils::PointIRLCloud &cloud, int label,
                                  std::vector<int> *indices) {
  CHECK(indices != nullptr);
  indices->clear();

  const int cloud_size = cloud.size();
  for (int i = 0; i < cloud_size; i++) {
    if (cloud.points[i].label != label) {
      continue;
    }

    indices->push_back(i);
  }
}

void ExtractSemanticPoints(const pcl_utils::PointIRLCloud &cloud, int label,
                           pcl_utils::PointICloud::Ptr cloud_out) {
  CHECK(cloud_out != nullptr);
  cloud_out->clear();

  for (const auto &pt : cloud.points) {
    if (pt.label != label) {
      continue;
    }
    pcl_utils::PointI point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = pt.label;
    cloud_out->push_back(point);
  }
}

pcl::PointXY Vector4fToPoint2d(const Eigen::Vector4f &vec) {
  pcl::PointXY pt;
  pt.x = vec.x();
  pt.y = vec.y();
  return pt;
}

} // namespace pcl_utils

// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/11.

#pragma once

#include <fstream>
#include <string>
#include <unordered_map>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <pcl/filters/filter.h>

#include "common/pcl_utils/pcl_types.h"

DECLARE_bool(is_lidar_data_with_ring);

namespace long_term_relocalization {

pcl_utils::PointIRLCloud ReadVelodyneBinFile(const std::string &scan_file_name);

std::vector<uint32_t> ReadLabelBinFile(const std::string &label_file_name);

pcl_utils::PointIRLCloud ReadVelodyneAndLabelBinFiles(const std::string &scan_file_name,
                                                      const std::string &label_file_name);

std::unordered_map<int, int> MapRingFrom80To64();

template <typename PointT>
void WriteRS80LidarDataToBinFile(const pcl::PointCloud<PointT> &cloud_in,
                                 const std::string &scan_file_name) {
  std::ofstream out(scan_file_name.c_str(), std::ios::binary | std::ios::out);
  CHECK(out.is_open());

  pcl::PointCloud<PointT> cloud;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_in, cloud, indices);

  const int size_of_point = FLAGS_is_lidar_data_with_ring ? 5 : 4;

  const uint32_t num_points = cloud.size();
  std::vector<float> values(size_of_point * num_points);
  for (uint32_t i = 0; i < num_points; ++i) {
    values[size_of_point * i] = cloud.points[i].x;
    values[size_of_point * i + 1] = cloud.points[i].y;
    values[size_of_point * i + 2] = cloud.points[i].z;
    values[size_of_point * i + 3] = cloud.points[i].intensity;
    if (FLAGS_is_lidar_data_with_ring) {
      values[size_of_point * i + 4] = cloud.points[i].ring;
    }
  }
  out.write((char *)&values[0], size_of_point * num_points * sizeof(float));
  out.close();
}

} // namespace long_term_relocalization

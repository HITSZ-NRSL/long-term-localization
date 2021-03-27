// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/11.

#include "long_term_relocalization/offline_process/semantic_kitti_utils.h"

#include <unordered_map>
#include <unordered_set>

#include "common/pcl_utils/pcl_utils.h"

// we write {x,y,z,intensity,ring} to .bin file
DEFINE_bool(is_lidar_data_with_ring, true, "write lidar data with ring or not.");

namespace long_term_relocalization {

// clang-format off
const std::unordered_map<int, int> label_remap = {
  // {label, intensity}
  // {0, 10},   // "unlabeled"
  // {1, 20},   // "outlier"
  {10, 30},  // "car"
  {11, 180},  // "bicycle"
  {13, 50},  // "bus"
  {15, 60},  // "motorcycle"
  {18, 80},  // "truck"
  {20, 90},  // "other-vehicle"
  {30, 150}, // "person"
  {31, 120}, // "bicyclist"
  {32, 130}, // "motorcyclist"
  {40, 10},  // "road" red
  {48, 160}, // "sidewalk"
  {49, 170}, // "other-ground"
  {50, 40},  // "building", yellow
  {70, 110}, // "vegetation"
  {71, 230}, // "trunk"
  {72, 120}, // "terrain"
  {80, 250}, // "pole"
  {81, 70},  // "traffic-sign"
  {99, 190}   // "other-object"
};
// clang-format on

pcl_utils::PointIRLCloud ReadVelodyneBinFile(const std::string &scan_file_name) {
  CHECK(boost::filesystem::exists(scan_file_name)) << scan_file_name << " is not exist.";
  const int size_of_point = FLAGS_is_lidar_data_with_ring ? 5 : 4;

  // Open a scan
  std::ifstream in(scan_file_name.c_str(), std::ios::binary);
  CHECK(in.is_open());

  in.seekg(0, std::ios::end);
  const uint32_t num_points = in.tellg() / (size_of_point * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(size_of_point * num_points);
  in.read((char *)&values[0], size_of_point * num_points * sizeof(float));
  in.close();

  pcl_utils::PointIRLCloud cloud;
  cloud.reserve(num_points);
  for (int i = 0; i < num_points; i++) {
    pcl_utils::PointIRL point;
    point.x = values[size_of_point * i];
    point.y = values[size_of_point * i + 1];
    point.z = values[size_of_point * i + 2];
    point.intensity = values[size_of_point * i + 3];
    if (FLAGS_is_lidar_data_with_ring) {
      point.ring = values[size_of_point * i + 4];
    }
    cloud.push_back(point);
  }

  return cloud;
}

std::vector<uint32_t> ReadLabelBinFile(const std::string &label_file_name) {
  CHECK(boost::filesystem::exists(label_file_name)) << label_file_name << " is not exist.";

  std::ifstream in(label_file_name.c_str(), std::ios::binary);
  CHECK(in.is_open());

  in.seekg(0, std::ios::end);
  const uint32_t num_points = in.tellg() / (sizeof(uint32_t));
  in.seekg(0, std::ios::beg);

  std::vector<uint32_t> labels(num_points);
  in.read((char *)&labels[0], num_points * sizeof(uint32_t));
  in.close();

  return labels;
}

pcl_utils::PointIRLCloud ReadVelodyneAndLabelBinFiles(const std::string &scan_file_name,
                                                      const std::string &label_file_name) {
  pcl_utils::PointIRLCloud cloud = ReadVelodyneBinFile(scan_file_name);
  const std::vector<uint32_t> labels = ReadLabelBinFile(label_file_name);
  CHECK_EQ(cloud.size(), labels.size());

  pcl_utils::PointIRLCloud cloud_out;
  cloud_out.reserve(cloud.size());
  for (int i = 0; i < cloud.size(); ++i) {
    auto &point = cloud.points[i];
    const int label = labels[i] & 0x00FF;
    if (label_remap.count(label) == 0) {
      continue;
    }

    // TODO(silin): visualize semantic labels
    point.intensity = label_remap.at(label);
    point.label = label;
    cloud_out.push_back(point);
  }

  return cloud_out;
}

std::unordered_map<int, int> MapRingFrom80To64() {
  std::unordered_set<int> outliers;
  for (int i = 0; i < 16; ++i) {
    outliers.insert(i * 2 + 7);
  }

  std::unordered_map<int, int> indices_map;
  int new_index = 0;
  for (int old_index = 0; old_index < 80; ++old_index) {
    if (outliers.count(old_index) > 0) {
      continue;
    }
    indices_map[old_index] = new_index;
    ++new_index;
  }

  CHECK_EQ(indices_map.size(), 64);

  return indices_map;
}

} // namespace long_term_relocalization

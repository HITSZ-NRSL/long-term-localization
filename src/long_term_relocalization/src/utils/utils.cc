// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/26.

#include "long_term_relocalization/utils/utils.h"

#include <ros/package.h>

#include "common/file/file_path.h"

namespace long_term_relocalization {

std::string GetCurrentPackagePath() { return ros::package::getPath(kCurrentPackageName); }

std::string GetDataDirectoryPath() {
  return file::PathJoin(GetCurrentPackagePath(), kRelativeDataDirPath);
}

std::string GetMapDirectoryPath() {
  return file::PathJoin(GetCurrentPackagePath(), kRelativeMapDirPath);
}

std::string GetConfigFilePath() {
  return file::PathJoin(GetCurrentPackagePath(), kRelativeParamsFilePath);
}

std::string GetRS80AngleFilePath() {
  return file::PathJoin(GetCurrentPackagePath(), kRelativeRs80AngleFilePath);
}

std::set<int> RandomDownsamplingArray(int max_size, double remain_ratio /*[0, 1]*/) {
  CHECK_GT(max_size, 0);
  CHECK_GE(remain_ratio, 0);
  CHECK_LE(remain_ratio, 1);

  std::vector<int> indices(max_size);
  std::iota(indices.begin(), indices.end(), 0);

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::shuffle(indices.begin(), indices.end(), std::default_random_engine(seed));

  std::set<int> downsampled_indices;
  const int end_index = max_size * remain_ratio;
  for (int i = 0; i < end_index; ++i) {
    downsampled_indices.insert(indices[i]);
  }

  return downsampled_indices;
}

} // namespace long_term_relocalization

// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/4.

#include "common/file/file_path.h"

namespace file {
std::string PathJoin(const std::string &base_path, const std::string &name) {
  return (boost::filesystem::path(base_path) / name).string();
}

std::vector<std::string> ListFiles(const std::string &directory_name) {
  CHECK(boost::filesystem::is_directory(directory_name))
      << directory_name << " is not a directory.";

  std::vector<std::string> file_names;
  for (const auto &file_name :
       boost::make_iterator_range(boost::filesystem::directory_iterator(directory_name))) {
    file_names.emplace_back(file_name.path().string());
  }
  std::sort(file_names.begin(), file_names.end());
  return file_names;
}
} // namespace file

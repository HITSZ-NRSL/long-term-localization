// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/4.

#pragma once

#include <string>
#include <vector>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

namespace file {

std::string PathJoin(const std::string &base_path, const std::string &name);

std::vector<std::string> ListFiles(const std::string &directory_name);

} // namespace file

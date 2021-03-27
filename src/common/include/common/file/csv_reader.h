// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/11.

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <absl/strings/str_split.h>
#include <absl/strings/string_view.h>

namespace file {

class CsvReader {
public:
  CsvReader(const std::string &filename, char delimiter);
  virtual ~CsvReader() = default;

  bool GetLine(std::vector<std::string> *strs);
  bool ParseLine(std::vector<double> *data);
  bool ParseLine(std::vector<uint64_t> *data);
  bool ParseLine(uint64_t *timestamp, std::vector<double> *data);

private:
  std::ifstream input_file_;
  char delimiter_ = 0;
};

} // namespace file

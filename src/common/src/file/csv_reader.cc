// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/11.

#include "common/file/csv_reader.h"

namespace file {

CsvReader::CsvReader(const std::string &filename, char delimiter) : delimiter_(delimiter) {
  CHECK(boost::filesystem::is_regular_file(filename)) << filename << " is not exist.";
  input_file_.open(filename);
  CHECK(input_file_.is_open() && input_file_.good());
}

bool CsvReader::GetLine(std::vector<std::string> *strs) {
  CHECK(strs != nullptr);

  constexpr int kMaxLineSize = 200;
  char line[kMaxLineSize];

  if (!input_file_.getline(line, kMaxLineSize)) {
    return false;
  }

  *strs = absl::StrSplit(absl::string_view(line), delimiter_);
  return true;
}

bool CsvReader::ParseLine(std::vector<double> *data) {
  CHECK(data != nullptr);
  data->clear();

  std::vector<std::string> strs;
  if (!GetLine(&strs)) {
    return false;
  }

  for (int i = 0; i < strs.size(); ++i) {
    (*data).emplace_back(std::stod(strs[i]));
  }
  return true;
}

bool CsvReader::ParseLine(std::vector<uint64_t> *data) {
  CHECK(data != nullptr);
  data->clear();

  std::vector<std::string> strs;
  if (!GetLine(&strs)) {
    return false;
  }

  for (int i = 0; i < strs.size(); ++i) {
    (*data).emplace_back(std::stoul(strs[i]));
  }
  return true;
}

bool CsvReader::ParseLine(uint64_t *timestamp, std::vector<double> *data) {
  CHECK(timestamp != nullptr && data != nullptr);
  data->clear();

  std::vector<std::string> strs;
  if (!GetLine(&strs)) {
    return false;
  }

  *timestamp = std::stoul(strs.front());

  for (int i = 1; i < strs.size(); ++i) {
    (*data).emplace_back(std::stod(strs[i]));
  }
  return true;
}

} // namespace file

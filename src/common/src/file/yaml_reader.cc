// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#include "common/file/yaml_reader.h"

namespace file {

YamlReader::YamlReader(const std::string &config_file) {
  CHECK(boost::filesystem::exists(config_file));
  node_ = YAML::LoadFile(config_file);
}

YamlReader::YamlReader(const YAML::Node &node) : node_(std::move(node)) {}

} // namespace file
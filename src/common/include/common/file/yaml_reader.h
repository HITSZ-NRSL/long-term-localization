// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#pragma once

#include <memory>

#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <yaml-cpp/yaml.h>

namespace file {

class YamlReader {
public:
  explicit YamlReader(const std::string &config_file);
  explicit YamlReader(const YAML::Node &node);

  virtual ~YamlReader() = default;

  template <typename T> inline T ReadValue(const std::string &key) {
    CHECK(node_[key].Type() != YAML::NodeType::Null);
    CHECK(node_[key]) << key << " is not exist!";

    return node_[key].as<T>();
  }

  inline YAML::Node ReadSubNode(const std::string &subnode_name) {
    YAML::Node ret = node_[subnode_name.c_str()];
    CHECK_EQ(!ret, false) << "Cannot find subnode " << subnode_name;
    return ret;
  }

private:
  YAML::Node node_;
};

} // namespace file

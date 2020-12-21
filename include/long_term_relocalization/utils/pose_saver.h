// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-11-26.

#pragma once

#include <fstream>
#include <string>

#include "common/pcl_utils/pcl_types.h"
#include "common/ros_utils/ros_utils.h"

namespace long_term_relocalization {

class PoseSaver {
public:
  explicit PoseSaver(const std::string &filename);

  ~PoseSaver();

  void Write(const pcl_utils::Point &point, const ros::Time &stamp);
  void Write(const tf::Transform &transform, const ros::Time &stamp);
  void Write(const nav_msgs::Odometry &odometry);

private:
  std::ofstream outfile_;
};

} // namespace long_term_relocalization

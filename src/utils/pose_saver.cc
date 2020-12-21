// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-11-26.

#include "long_term_relocalization/utils/pose_saver.h"

#include <boost/filesystem.hpp>

namespace long_term_relocalization {

PoseSaver::PoseSaver(const std::string &filename) {
  outfile_.open(filename);
  CHECK(outfile_.is_open() && outfile_.good());
}

PoseSaver::~PoseSaver() { outfile_.close(); }

void PoseSaver::Write(const pcl_utils::Point &point, const ros::Time &stamp) {
  outfile_ << std::setprecision(16) << stamp.toSec() << " " << point.x << " " << point.y << " "
           << point.z << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
}

void PoseSaver::Write(const tf::Transform &transform, const ros::Time &stamp) {
  outfile_ << std::setprecision(16) << stamp.toSec() << " " << transform.getOrigin().x() << " "
           << transform.getOrigin().y() << " " << transform.getOrigin().z() << " "
           << transform.getRotation().x() << " " << transform.getRotation().y() << " "
           << transform.getRotation().z() << " " << transform.getRotation().w() << std::endl;
}

void PoseSaver::Write(const nav_msgs::Odometry &odometry) {
  outfile_ << std::setprecision(16) << odometry.header.stamp.toSec() << " "
           << odometry.pose.pose.position.x << " " << odometry.pose.pose.position.y << " "
           << odometry.pose.pose.position.z << " " << odometry.pose.pose.orientation.x << " "
           << odometry.pose.pose.orientation.y << " " << odometry.pose.pose.orientation.z << " "
           << odometry.pose.pose.orientation.w << std::endl;
}

} // namespace long_term_relocalization

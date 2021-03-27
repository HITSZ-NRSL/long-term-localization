// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/10.

#pragma once

#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace ros_utils {

// Read ground_true/point_cloud/imu from nclt dataset file, and write them to
// rosbag. Odom/PointCloud2/Imu
class BagWriter {
public:
  // enum BagMode
  // {
  //     Write   = 1,
  //     Read    = 2,
  //     Append  = 4
  // };
  BagWriter(const std::string &bag_file_name, uint32_t mode /*rosbag::bagmode::BagMode::Write*/) {
    bag_ = std::make_unique<rosbag::Bag>(bag_file_name, mode);
  }

  template <typename MessageType> void Write(const std::string &topic, const MessageType &msg) {
    bag_->write(topic, msg.header.stamp, msg);
  }

  virtual ~BagWriter() = default;

private:
  std::unique_ptr<rosbag::Bag> bag_;
};

} // namespace ros_utils

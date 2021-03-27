// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#pragma once

#include <opencv/cv.hpp>

#include <glog/logging.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include "kindr/minimal/quat-transformation.h"

namespace ros_utils {

template <typename PointType>
sensor_msgs::PointCloud2 ToCloudMsg(const pcl::PointCloud<PointType> &cloud,
                                    const std::string &frame_id, const ros::Time &stamp) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.header.stamp = stamp;

  return cloud_msg;
}

template <typename PointT>
pcl::PointCloud<PointT> FromCloudMsg(const sensor_msgs::PointCloud2 &cloud_msg) {
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);
  return cloud_msg;
}

sensor_msgs::Image ToImageMsg(const cv::Mat &image, const std::string &frame_id,
                              const ros::Time &time,
                              const std::string &encoding = sensor_msgs::image_encodings::BGR8);

cv::Mat FromImageMsg(const sensor_msgs::Image &image, const std::string &image_encoding);

nav_msgs::Odometry ToOdomMsg(const kindr::minimal::QuatTransformation &quat_transform,
                             const std::string &frame_id, const std::string &child_frame_id,
                             const ros::Time &stamp);

} // namespace ros_utils

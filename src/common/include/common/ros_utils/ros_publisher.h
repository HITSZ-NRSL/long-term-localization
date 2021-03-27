// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/7/27.

#pragma once

#include <opencv/cv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "common/ros_utils/ros_utils.h"

namespace ros_utils {
// NOTE: MsyType must be without Ptr or ConstPtr, e.g. sensor_msgs::PointCloud2
template <typename MsyType> class RosPublisher {
public:
  RosPublisher(const std::string &topic, uint32_t queue_size, const std::string &frame_id,
               const std::string &child_frame_id = "", bool latch = false)
      : nh_("~"), frame_id_(frame_id), child_frame_id_(child_frame_id) {
    LOG(INFO) << "RosPublisher ---> " << topic;
    pub_ = nh_.advertise<MsyType>(topic, queue_size, latch);
  }

  void Publish(const MsyType &msg) { pub_.publish(msg); }

  template <typename OctomapT> void Publish(const OctomapT &octomap, const ros::Time &timestamp) {
    octomap_msgs::Octomap octree_msg;
    octomap_msgs::fullMapToMsg(octomap, octree_msg);
    octree_msg.header.frame_id = frame_id_;
    octree_msg.header.stamp = timestamp;
    pub_.publish(octree_msg);
  }

  template <typename PointType>
  void Publish(const pcl::PointCloud<PointType> &cloud, const ros::Time &timestamp) {
    const sensor_msgs::PointCloud2 cloud_msg =
        ros_utils::ToCloudMsg(cloud, frame_id_, timestamp);
    Publish(cloud_msg);
  }

  void Publish(const cv::Mat &image, const ros::Time &timestamp) {
    const sensor_msgs::Image image_msg = ros_utils::ToImageMsg(image, frame_id_, timestamp);
    Publish(image_msg);
  }

  void Publish(const kindr::minimal::QuatTransformation &quat_transform,
               const ros::Time &timestamp) {
    const nav_msgs::Odometry odom_msg =
        ros_utils::ToOdomMsg(quat_transform, frame_id_, child_frame_id_, timestamp);
    Publish(odom_msg);
  }

  bool HasSubscribers() const { return pub_.getNumSubscribers() > 0; }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string frame_id_;
  std::string child_frame_id_;
};

} // namespace ros_utils

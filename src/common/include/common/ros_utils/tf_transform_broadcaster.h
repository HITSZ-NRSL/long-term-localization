// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-8-31.

#pragma once

#include <memory>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "kindr/minimal/quat-transformation.h"
#include "common/ros_utils/ros_utils.h"

namespace ros_utils {

class TfTransformBroadcaster {
public:
  TfTransformBroadcaster(const std::string &frame_id, const std::string &child_frame_id);
  virtual ~TfTransformBroadcaster() = default;

  void Publish(const kindr::minimal::QuatTransformation &quat_transform, const ros::Time &stamp);
  void Publish(const tf::Transform &transform, const ros::Time &stamp);
  void Publish(const nav_msgs::Odometry &odometry);

private:
  std::string frame_id_;
  std::string child_frame_id_;
  std::unique_ptr<tf::TransformBroadcaster> tf_tb_;
};

class TfStaticTransformBroadcaster {
public:
  TfStaticTransformBroadcaster(const kindr::minimal::QuatTransformation &quat_transform,
                               const std::string &frame_id, const std::string &child_frame_id);
  virtual ~TfStaticTransformBroadcaster() = default;
  void Publish(const ros::Time &stamp);

private:
  kindr::minimal::QuatTransformation transform_;
  std::string frame_id_;
  std::string child_frame_id_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<geometry_msgs::TransformStamped> static_transformStamped_;
};
} // namespace ros_utils

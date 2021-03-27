// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-8-31.

#include "common/ros_utils/tf_transform_broadcaster.h"

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"

namespace ros_utils {

TfTransformBroadcaster::TfTransformBroadcaster(const std::string &frame_id,
                                               const std::string &child_frame_id)
    : frame_id_(frame_id), child_frame_id_(child_frame_id) {
  tf_tb_ = std::make_unique<tf::TransformBroadcaster>();
}

void TfTransformBroadcaster::Publish(const kindr::minimal::QuatTransformation &quat_transform,
                                     const ros::Time &stamp) {
  tf::Transform transform;
  minkindr_conversions::transformKindrToTF(quat_transform, &transform);
  tf_tb_->sendTransform(tf::StampedTransform(transform, stamp, frame_id_, child_frame_id_));
}

void TfTransformBroadcaster::Publish(const tf::Transform &transform, const ros::Time &stamp) {
  tf_tb_->sendTransform(tf::StampedTransform(transform, stamp, frame_id_, child_frame_id_));
}

void TfTransformBroadcaster::Publish(const nav_msgs::Odometry &odometry) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y,
                                  odometry.pose.pose.position.z));
  tf::Quaternion q(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,
                   odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w);
  transform.setRotation(q);

  tf_tb_->sendTransform(
      tf::StampedTransform(transform, odometry.header.stamp, frame_id_, child_frame_id_));
}

TfStaticTransformBroadcaster::TfStaticTransformBroadcaster(
    const kindr::minimal::QuatTransformation &quat_transform, const std::string &frame_id,
    const std::string &child_frame_id)
    : transform_(quat_transform), frame_id_(frame_id), child_frame_id_(child_frame_id) {
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
  static_transformStamped_ = std::make_unique<geometry_msgs::TransformStamped>();
}

void TfStaticTransformBroadcaster::Publish(const ros::Time &stamp) {
  static_transformStamped_->header.stamp = stamp;
  static_transformStamped_->header.frame_id = frame_id_;
  static_transformStamped_->child_frame_id = child_frame_id_;

  minkindr_conversions::transformKindrToMsg(transform_, &static_transformStamped_->transform);
  static_broadcaster_->sendTransform(*static_transformStamped_);
}

} // namespace ros_utils

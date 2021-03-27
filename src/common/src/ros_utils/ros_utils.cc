// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#include "common/ros_utils/ros_utils.h"

#include "minkindr_conversions/kindr_msg.h"

namespace ros_utils {

sensor_msgs::Image ToImageMsg(const cv::Mat &image, const std::string &frame_id,
                              const ros::Time &stamp, const std::string &encoding) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = stamp;
  cv_bridge::CvImage cv_image(header, encoding, image);
  return *cv_image.toImageMsg();
}

cv::Mat FromImageMsg(const sensor_msgs::Image &image, const std::string &image_encoding) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, image_encoding);
  cv::Mat img = cv_ptr->image;
  return img;
}

nav_msgs::Odometry ToOdomMsg(const kindr::minimal::QuatTransformation &quat_transform,
                             const std::string &frame_id, const std::string &child_frame_id,
                             const ros::Time &stamp) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = frame_id;
  odom_msg.header.stamp = stamp;
  odom_msg.child_frame_id = child_frame_id;

  minkindr_conversions::poseKindrToMsg(quat_transform, &odom_msg.pose.pose);
  return odom_msg;
}

} // namespace ros_utils

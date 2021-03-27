// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/20.

#include "long_term_relocalization/offline_process/semantic_kitti_utils.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "common/file/csv_reader.h"
#include "common/file/file_path.h"
#include "common/ros_utils/bag_writer.h"
#include "common/ros_utils/ros_publisher.h"
#include "common/ros_utils/tf_transform_broadcaster.h"
#include "long_term_relocalization/utils/constants.h"

DEFINE_string(dataset_dir, "", "dir contains labels,velodyne,semantic.bag directorys.");

using namespace long_term_relocalization;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "semantic_bag_writer_main");

  // Init bag writer.
  const std::string bag_file_path = file::PathJoin(FLAGS_dataset_dir, "semantic.bag");
  CHECK(boost::filesystem::is_regular_file(bag_file_path)) << bag_file_path << " is not exist.";
  ros_utils::BagWriter bag_writer(bag_file_path, rosbag::bagmode::BagMode::Append);

  CHECK(boost::filesystem::exists(FLAGS_dataset_dir)) << FLAGS_dataset_dir;
  const std::vector<std::string> velo_files =
      file::ListFiles(file::PathJoin(FLAGS_dataset_dir, "velodyne"));
  const std::vector<std::string> label_files =
      file::ListFiles(file::PathJoin(FLAGS_dataset_dir, "labels"));
  CHECK_EQ(velo_files.size(), label_files.size());

  // Read timestamp.
  const std::string stamp_file = file::PathJoin(FLAGS_dataset_dir, "times.txt");
  file::CsvReader stamp_csv_reader(stamp_file, ' ');
  std::vector<uint64_t> stamp_data;
  std::vector<ros::Time> stamps;
  while (stamp_csv_reader.ParseLine(&stamp_data)) {
    CHECK_EQ(stamp_data.size(), 1);
    ros::Time t;

    t.fromNSec(stamp_data.front());
    stamps.emplace_back(t);
  }

  // Read cloud poses.
  const std::string pose_file = file::PathJoin(FLAGS_dataset_dir, "poses.txt");
  file::CsvReader pose_csv_reader(pose_file, ' ');
  std::vector<double> pose_data;
  std::vector<nav_msgs::Odometry> poses;
  while (pose_csv_reader.ParseLine(&pose_data)) {
    poses.emplace_back();
    auto &pose = poses.back();

    Eigen::Matrix3d mat;
    // clang-format off
    mat << pose_data[0], pose_data[1], pose_data[2],
           pose_data[4], pose_data[5], pose_data[6],
           pose_data[8], pose_data[9], pose_data[10];
    // clang-format on
    const Eigen::Quaterniond q(mat);

    pose.pose.pose.position.x = pose_data[3];
    pose.pose.pose.position.y = pose_data[7];
    pose.pose.pose.position.z = pose_data[11];
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.orientation.w = q.w();
    pose.child_frame_id = kLidarFrameId;
    pose.header.frame_id = kOdomFrameId;
  }
  CHECK_EQ(stamps.size(), velo_files.size());
  CHECK_EQ(stamps.size(), poses.size());

  for (int i = 0; i < velo_files.size() && ros::ok(); ++i) {
    const std::string velo_filename = boost::filesystem::path(velo_files[i]).filename().string();
    const std::string label_filename = boost::filesystem::path(label_files[i]).filename().string();
    CHECK_EQ(velo_filename.substr(0, 6), label_filename.substr(0, 6));

    const pcl_utils::PointIRLCloud cloud =
        ReadVelodyneAndLabelBinFiles(velo_files[i], label_files[i]);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = kLidarFrameId;

    LOG(INFO) << std::fixed << stamps[i].toNSec();

    cloud_msg.header.stamp = stamps[i];
    bag_writer.Write(kSemanticLidarPointsTopic, cloud_msg);

    poses[i].header.stamp = stamps[i];
    bag_writer.Write(kSemanticLidarPoseTopic, poses[i]);

    LOG(INFO) << "processed: " << velo_filename << "/" << velo_files.size();
  }

  return 0;
}

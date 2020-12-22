// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/19.

#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <Eigen/Dense>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/flann.h>

#include <boost/filesystem.hpp>

#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "common/file/file_path.h"
#include "common/pcl_utils/pcl_types.h"
#include "common/pcl_utils/pcl_utils.h"
#include "common/ros_utils/ros_publisher.h"
#include "common/ros_utils/ros_utils.h"
#include "common/ros_utils/tf_transform_broadcaster.h"
#include "common/tic_toc.h"
#include "long_term_relocalization/depth_clustering/depth_clustering.h"
#include "long_term_relocalization/recognizers/correspondence_recognizer_factory.h"
#include "long_term_relocalization/relocalization/cluster_manager.h"
#include "long_term_relocalization/relocalization/keyframe.h"
#include "long_term_relocalization/relocalization/semantic_cluster_map.h"
#include "long_term_relocalization/utils/params_types.h"
#include "long_term_relocalization/utils/pose_saver.h"

namespace long_term_relocalization {

class Relocalization {
  using Point2d = Cluster::Point2d;
  using PointT = Cluster::PointT;
  using PointCloud = Cluster::PointCloud;
  using KeyFramePtr = KeyFrame<PointT>::Ptr;
  using ApproximateTimePolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>;

public:
  Relocalization(const RelocalizationParams &params, const ros::NodeHandle &nh);
  virtual ~Relocalization();

private:
  void Initialize();

  void InitializeTargetClusterMap();

  void PublishTargetClusterMap();

  void CloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                         const nav_msgs::OdometryConstPtr &pose);

  void LaserOdometryCallback(const nav_msgs::OdometryConstPtr &odom);

  void UpdatePathAndPublish(const pcl_utils::Point &map_pose, const ros::Time &stamp);

  void SaveMapCmdCallback(const std_msgs::Int32ConstPtr &msg);

  void GlobalMapHandlerThread();
  void SemanticClusterMapThread();
  void RelocalizationThread();

  PairwiseMatches RunSemanticClusterMatcher();

  void PublisherMatches(const PairwiseMatches &matches);
  void PublishLineSet(const PointPairs &point_pairs, const float line_scale, const Color &color);

  ros::NodeHandle nh_;
  RelocalizationParams params_;

  ros_utils::RosPublisher<sensor_msgs::PointCloud2> source_point_cloud_map_publisher_;
  ros_utils::RosPublisher<sensor_msgs::PointCloud2> source_clusters_cloud_publisher_;
  ros_utils::RosPublisher<sensor_msgs::PointCloud2> source_clusters_centroids_publisher_;
  ros_utils::RosPublisher<sensor_msgs::PointCloud2> target_clusters_cloud_publisher_;
  ros_utils::RosPublisher<sensor_msgs::PointCloud2> target_clusters_centroids_publisher_;

  ros_utils::TfTransformBroadcaster tf_odom_lidar_publisher_;
  ros_utils::TfTransformBroadcaster tf_map_odom_publisher_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *pose_sub_;
  message_filters::Synchronizer<ApproximateTimePolicy> *sync_;

  ros::Subscriber save_map_cmd_sub_;
  ros::Subscriber odometry_sub_;
  ros::Publisher matches_pub_;
  ros::Publisher localization_path_pub_;

  nav_msgs::Path localization_path_;
  std::unique_ptr<PoseSaver> localization_pose_saver_;
  std::unique_ptr<PoseSaver> relocalization_pose_saver_;
  std::unique_ptr<PoseSaver> gt_pose_saver_;

  // Preventing data racing!
  std::mutex keyframes_mutex_;
  std::mutex local_keyframes_mutex_;
  std::mutex source_point_cloud_map_mutex_;
  std::mutex semantic_cluster_map_mutex_;

  std::queue<KeyFramePtr> keyframes_;
  std::condition_variable keyframes_condition_;
  std::condition_variable run_matcher_condition_;

  ClustersManager target_clusters_manager_;
  pcl::KdTreeFLANN<Cluster::Point2d>::Ptr target_kdtree_;
  Eigen::Matrix4f map_lidar_transform_;
  bool is_relocalization_successful_ = false;

  std::unique_ptr<SemanticClusterMap> semantic_cluster_map_;
  std::unique_ptr<ClusterMatcher> cluster_matcher_;
  std::unique_ptr<CorrespondenceRecognizer> recognizer_;

  std::unique_ptr<std::thread> semantic_cluster_map_thread_;
  std::unique_ptr<std::thread> relocalization_thread_;

  //-------- For Evalute --------
  bool have_recorded_relozlization_distance_ = false;
  Point2d *first_local_pose_ = nullptr;
};

} // namespace long_term_relocalization

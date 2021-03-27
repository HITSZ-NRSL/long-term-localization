// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-10-09.

#include "long_term_relocalization/depth_clustering/depth_clustering.h"

#include "common/file/file_path.h"
#include "common/ros_utils/ros_publisher.h"
#include "common/ros_utils/ros_utils.h"
#include "long_term_relocalization/depth_clustering/range_image_constructor.h"
#include "long_term_relocalization/utils/constants.h"
#include "long_term_relocalization/utils/params_loader.h"
#include "long_term_relocalization/utils/utils.h"

using namespace long_term_relocalization;

std::unique_ptr<ParamsLoader> params_loader;
std::unique_ptr<DepthClustering<pcl_utils::PointIRL>> depth_clustering_;
std::unique_ptr<ros_utils::RosPublisher<sensor_msgs::PointCloud2>> ground_cloud_publisher_;
std::unique_ptr<ros_utils::RosPublisher<sensor_msgs::PointCloud2>> cluster_cloud_publisher_;
std::unique_ptr<ros_utils::RosPublisher<sensor_msgs::PointCloud2>> trunk_pole_cloud_publisher_;

//=====================================================
// Utils function declaration
//=====================================================
std::vector<pcl_utils::PointIRLCloud::Ptr>
FilterClusters(const std::vector<pcl_utils::PointIRLCloud::Ptr> &clusters);
bool IsPoleLikeCluster(const pcl_utils::PointIRLCloud &cluster);
pcl_utils::PointIRLCloud
ExtractClusterPoints(const std::vector<pcl_utils::PointIRLCloud::Ptr> &clusters);

void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl_utils::PointIRLCloud cloud;
  pcl::fromROSMsg(*msg, cloud);

  depth_clustering_->Process(cloud);
  const std::vector<pcl_utils::PointIRLCloud::Ptr> clusters = depth_clustering_->ExtractClusters();
  const std::vector<pcl_utils::PointIRLCloud::Ptr> filtered_clusters = FilterClusters(clusters);

  ground_cloud_publisher_->Publish(depth_clustering_->ExtractGroundPoints(), msg->header.stamp);
  cluster_cloud_publisher_->Publish(depth_clustering_->ExtractClusterPoints(), msg->header.stamp);
  trunk_pole_cloud_publisher_->Publish(ExtractClusterPoints(filtered_clusters), msg->header.stamp);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_clustering_main");
  ros::NodeHandle nh("~");
  ros::Subscriber rslidar_points_sub =
      nh.subscribe<sensor_msgs::PointCloud2>("/semantic_points", 5, &PointsCallback);

  params_loader = std::make_unique<ParamsLoader>(GetConfigFilePath(), GetRS80AngleFilePath());

  const DepthClusteringParams &depth_clustering_params =
      params_loader->get_depth_clustering_params();

  depth_clustering_ =
      std::make_unique<DepthClustering<pcl_utils::PointIRL>>(depth_clustering_params);

  ground_cloud_publisher_ = std::make_unique<ros_utils::RosPublisher<sensor_msgs::PointCloud2>>(
      "ground_cloud", 5, "base_link", "", true);
  cluster_cloud_publisher_ = std::make_unique<ros_utils::RosPublisher<sensor_msgs::PointCloud2>>(
      "cluster_cloud", 5, "base_link", "", true);
  trunk_pole_cloud_publisher_ = std::make_unique<ros_utils::RosPublisher<sensor_msgs::PointCloud2>>(
      "trunk_pole_cloud", 5, "base_link", "", true);

  ros::spin();
  return 0;
}

//=====================================================
// Utils functions implementation
//=====================================================
std::vector<pcl_utils::PointIRLCloud::Ptr>
FilterClusters(const std::vector<pcl_utils::PointIRLCloud::Ptr> &clusters) {
  std::vector<pcl_utils::PointIRLCloud::Ptr> filtered_clusters;
  filtered_clusters.reserve(clusters.size());

  for (const auto &cluster : clusters) {
    pcl_utils::PointIRLCloud::Ptr cloud(new pcl_utils::PointIRLCloud());
    cloud->reserve(cluster->size());
    // for (const auto &point : cluster->points) {
    //   // Choose static semantic object points.
    //   if ((point.label == 80 || point.label == 71) && point.z <= 10.0) {
    //     cloud->push_back(point);
    //   }
    // }
    if (IsPoleLikeCluster(*cluster)) {
      filtered_clusters.push_back(cluster);
    }
  }

  return filtered_clusters;
}

bool IsPoleLikeCluster(const pcl_utils::PointIRLCloud &cluster) {
  // 1. filter by size
  if (cluster.size() <= 30) {
    return false;
  }

  // 2. filter by label count
  int trunk_count = 0;
  int pole_count = 0;
  for (const auto &point : cluster) {
    if (point.label == 80) {
      ++pole_count;
    } else if (point.label == 71) {
      ++trunk_count;
    }
  }

  const int wanted_label_count = pole_count + trunk_count;
  // const int wanted_label_count = std::max(pole_count, trunk_count);
  if (wanted_label_count <= 5) { // cluster.size() * 0.1
    return false;
  }

  // 3. filter by geometric conditions.
  Eigen::Vector4f min_point, max_point;
  pcl::getMinMax3D(cluster, min_point, max_point);

  const Eigen::Vector4f diff_point = max_point - min_point;
  const double xy_bound =
      std::sqrt(diff_point.x() * diff_point.x() + diff_point.y() * diff_point.y());
  const double height = std::abs(max_point.z() - min_point.z());

  return xy_bound <= 0.6 && height > 0.5;
}

pcl_utils::PointIRLCloud
ExtractClusterPoints(const std::vector<pcl_utils::PointIRLCloud::Ptr> &clusters) {
  pcl_utils::PointIRLCloud cloud;
  for (const auto &cluster : clusters) {
    cloud += *cluster;
  }

  return cloud;
}
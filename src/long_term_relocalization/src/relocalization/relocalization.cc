// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/19.

#include "long_term_relocalization/relocalization/relocalization.h"

#include "long_term_relocalization/utils/constants.h"

namespace long_term_relocalization {

Relocalization::Relocalization(const RelocalizationParams &params, const ros::NodeHandle &nh)
    : params_(params), nh_(nh),
      source_point_cloud_map_publisher_(kSourcePointCloudMapTopic, 5, kOdomFrameId),
      source_clusters_cloud_publisher_(kSourceClustersCloudTopic, 5, kOdomFrameId),
      source_clusters_centroids_publisher_(kSourceClustersCentroidsTopic, 5, kOdomFrameId),
      target_clusters_cloud_publisher_(kTargetClustersCloudTopic, 5, kMapFrameId, "", true),
      target_clusters_centroids_publisher_(kTargetClustersCentroidsTopic, 5, kMapFrameId, "", true),
      tf_odom_lidar_publisher_(kOdomFrameId, kLidarFrameId),
      tf_map_odom_publisher_(kMapFrameId, kOdomFrameId) {
  Initialize();
}

Relocalization::~Relocalization() {
  semantic_cluster_map_thread_->join();
  relocalization_thread_->join();
  delete cloud_sub_;
  delete pose_sub_;
  delete sync_;
}

void Relocalization::Initialize() {
  cloud_sub_ =
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, kSemanticLidarPointsTopic, 1);
  pose_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, kSemanticLidarPoseTopic, 1);
  sync_ = new message_filters::Synchronizer<ApproximateTimePolicy>(ApproximateTimePolicy(10),
                                                                   *cloud_sub_, *pose_sub_);
  sync_->registerCallback(boost::bind(&Relocalization::CloudPoseCallback, this, _1, _2));

  save_map_cmd_sub_ = nh_.subscribe<std_msgs::Int32>(kSaveMapCmdTopic, 1,
                                                     &Relocalization::SaveMapCmdCallback, this);

  odometry_sub_ = nh_.subscribe<nav_msgs::Odometry>(kLaserOdometryTopic, 1,
                                                    &Relocalization::LaserOdometryCallback, this);

  matches_pub_ = nh_.advertise<visualization_msgs::Marker>(kMatchesTopic, 5);

  localization_path_pub_ = nh_.advertise<nav_msgs::Path>(kLocalizationPathTopic, 5);

  LOG(INFO) << "subscribe cloud topic <== " << kSemanticLidarPointsTopic;
  LOG(INFO) << "subscribe pose topic <== " << kSemanticLidarPoseTopic;
  LOG(INFO) << "subscribe save map cmd topic <== " << kSaveMapCmdTopic;
  LOG(INFO) << "publish mathces ==> " << kMatchesTopic;

  semantic_cluster_map_ = std::make_unique<SemanticClusterMap>(params_.semantic_cluster_map_params);

  if (params_.mode == "relocalization") {
    InitializeTargetClusterMap();

    relocalization_thread_ =
        std::make_unique<std::thread>(&Relocalization::RelocalizationThread, this);

    cluster_matcher_ = std::make_unique<ClusterMatcher>(params_.cluster_matcher_params);

    CorrespondenceRecognizerFactory recognizer_factory(params_.geometric_consistency_params);
    recognizer_ = recognizer_factory.create();

    if (params_.evalute) {
      localization_pose_saver_ =
          std::make_unique<PoseSaver>(file::PathJoin(GetDataDirectoryPath(), "localization.txt"));
      relocalization_pose_saver_ =
          std::make_unique<PoseSaver>(file::PathJoin(GetDataDirectoryPath(), "relocalization.txt"));
      gt_pose_saver_ =
          std::make_unique<PoseSaver>(file::PathJoin(GetDataDirectoryPath(), "gt.txt"));
    }
  }

  semantic_cluster_map_thread_ =
      std::make_unique<std::thread>(&Relocalization::SemanticClusterMapThread, this);
}

void Relocalization::InitializeTargetClusterMap() {
  target_clusters_manager_.LoadClusters("/tmp/clusters_map.bin", params_.remain_ratio);
  SEGMENT_TIME_BEGIN(compute_target_clusters_descriptors);
  target_clusters_manager_.ComputeMiddleOutDescriptors(params_.search_radius);
  SEGMENT_TIME_END(compute_target_clusters_descriptors);
  LOG(INFO) << GREEN << "Target descriptors compute done" << COLOR_END;

  target_kdtree_ = boost::make_shared<pcl::KdTreeFLANN<Cluster::Point2d>>();
  target_kdtree_->setInputCloud(target_clusters_manager_.centroids2d_cloud());
}

void Relocalization::PublishTargetClusterMap() {
  static bool first = true;
  static PointCloud target_cluster_cloud;
  static PointCloud target_cluster_centroids;
  if (first) {
    target_cluster_cloud = *target_clusters_manager_.GetClustersCloud();
    pcl_utils::TranslateCloud(
        Eigen::Vector3d(0, 0, -params_.distance_to_lower_target_cloud_for_viz_m),
        &target_cluster_cloud);
    target_cluster_centroids = *target_clusters_manager_.centroids_cloud();
    pcl_utils::TranslateCloud(
        Eigen::Vector3d(0, 0, -params_.distance_to_lower_target_cloud_for_viz_m),
        &target_cluster_centroids);
    first = false;
  }
  target_clusters_cloud_publisher_.Publish(target_cluster_cloud, ros::Time::now());

  target_clusters_centroids_publisher_.Publish(target_cluster_centroids, ros::Time::now());
}

void Relocalization::CloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                       const nav_msgs::OdometryConstPtr &pose) {
  FUNC_TIME_BEGIN;

  pcl_utils::PointIRLCloud::Ptr semantic_cloud(new pcl_utils::PointIRLCloud());
  pcl::fromROSMsg(*cloud_msg, *semantic_cloud);

  const kindr::minimal::Position position(pose->pose.pose.position.x, pose->pose.pose.position.y,
                                          pose->pose.pose.position.z);
  const kindr::minimal::RotationQuaternion rotation(
      pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y,
      pose->pose.pose.orientation.z);
  const kindr::minimal::QuatTransformation trans(position, rotation);

  tf_odom_lidar_publisher_.Publish(*pose);

  KeyFramePtr keyframe = boost::make_shared<KeyFrame<pcl_utils::PointIRL>>(cloud_msg->header.stamp,
                                                                           trans, *semantic_cloud);

  std::unique_lock<std::mutex> lock1(keyframes_mutex_);
  keyframes_.push(keyframe);
  std::unique_lock<std::mutex> lock2(local_keyframes_mutex_);
  keyframes_.push(keyframe);
  if (keyframes_.size() >= params_.min_num_frames_in_local_map) {
    keyframes_condition_.notify_one();
  }

  if (gt_pose_saver_) {
    gt_pose_saver_->Write(*pose);
  }

  FUNC_TIME_END_WHETHER_OVERTIME(100);
}

void Relocalization::LaserOdometryCallback(const nav_msgs::OdometryConstPtr &odom) {
  if (!is_relocalization_successful_) {
    return;
  }

  const pcl_utils::Point odom_pose(odom->pose.pose.position.x, odom->pose.pose.position.y,
                                   odom->pose.pose.position.z);
  const Eigen::Transform<float, 3, Eigen::Affine> transform(map_lidar_transform_);
  const pcl_utils::Point map_pose = pcl::transformPoint(odom_pose, transform);

  UpdatePathAndPublish(map_pose, odom->header.stamp);

  // Save pose and evalute.
  if (localization_pose_saver_) {
    localization_pose_saver_->Write(map_pose, odom->header.stamp);
  }
}

void Relocalization::UpdatePathAndPublish(const pcl_utils::Point &map_pose,
                                          const ros::Time &stamp) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = stamp;
  pose_stamped.header.frame_id = kMapFrameId;
  pose_stamped.pose.position.x = map_pose.x;
  pose_stamped.pose.position.y = map_pose.y;
  pose_stamped.pose.position.z = map_pose.z;

  localization_path_.poses.push_back(pose_stamped);
  localization_path_.header.stamp = stamp;
  localization_path_.header.frame_id = kMapFrameId;
  localization_path_pub_.publish(localization_path_);
}

void Relocalization::SaveMapCmdCallback(const std_msgs::Int32ConstPtr &msg) {
  LOG(INFO) << GREEN << "msg data: " << msg->data << COLOR_END;
  if (msg->data == 1) {
    semantic_cluster_map_->mutable_cluster_manager()->SaveClustersToFile("/tmp/clusters_map.bin");
  }
}

void Relocalization::SemanticClusterMapThread() {
  std::queue<KeyFramePtr> keyframes;
  while (ros::ok()) {
    // TODO(silin): save the first kf pose, and transform all kf to the first frame.
    // 1. Transfer keyframes.
    std::unique_lock<std::mutex> local_keyframes_lock(local_keyframes_mutex_);
    keyframes_condition_.wait(local_keyframes_lock);
    while (!keyframes_.empty()) {
      keyframes.push(keyframes_.front());
      keyframes_.pop();
    }
    LOG_IF(ERROR, keyframes.size() > 10) << keyframes.size() << " frames in local map overflowed!";
    local_keyframes_lock.unlock();

    // 2. Build semantic local map
    SEGMENT_TIME_BEGIN(SemanticClusterMapThread);
    std::unique_lock<std::mutex> local_map_lock(semantic_cluster_map_mutex_);
    int process_cnt = 0;
    while (process_cnt++ <= params_.max_num_frames_in_local_map && !keyframes.empty()) {
      const KeyFramePtr &kf = keyframes.front();
      semantic_cluster_map_->UpdateKeyFrame(kf);
      keyframes.pop();
    }

    // 3. Publishers
    if (source_clusters_cloud_publisher_.HasSubscribers()) {
      const Cluster::PointCloud source_clusters_cloud =
          *semantic_cluster_map_->mutable_cluster_manager()->GetStaticClustersCloud();
      source_clusters_cloud_publisher_.Publish(source_clusters_cloud,
                                               semantic_cluster_map_->latest_stamp());
    }

    if (source_clusters_centroids_publisher_.HasSubscribers()) {
      const Cluster::PointCloud source_clusters_centroids =
          *semantic_cluster_map_->mutable_cluster_manager()->GetStaticCentroidsCloud();
      source_clusters_centroids_publisher_.Publish(source_clusters_centroids,
                                                   semantic_cluster_map_->latest_stamp());
    }

    run_matcher_condition_.notify_one();
    local_map_lock.unlock();
    SEGMENT_TIME_END_WHETHER_OVERTIME(SemanticClusterMapThread, 200);
  }
}

void Relocalization::RelocalizationThread() {
  LOG(INFO) << "***RelocalizationThread Start***";
  while (ros::ok()) {
    PublishTargetClusterMap();

    std::unique_lock<std::mutex> lock(semantic_cluster_map_mutex_);
    run_matcher_condition_.wait(lock);
    lock.unlock();

    const PairwiseMatches matches = RunSemanticClusterMatcher();
    const Eigen::Matrix4f transform = map_lidar_transform_;
    const Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
    const tf::Transform trans_map_odom(
        tf::Quaternion(q.x(), q.y(), q.z(), q.w()),
        tf::Vector3(transform(0, 3), transform(1, 3), transform(2, 3)));

    tf_map_odom_publisher_.Publish(trans_map_odom, semantic_cluster_map_->latest_stamp());

    if (!matches.empty()) {
      PublisherMatches(matches);
      if (relocalization_pose_saver_) {
        relocalization_pose_saver_->Write(trans_map_odom, semantic_cluster_map_->latest_stamp());
      }
    }
  }
}

PairwiseMatches Relocalization::RunSemanticClusterMatcher() {
  ClustersManager *source_cluster_manager = semantic_cluster_map_->mutable_cluster_manager();
  if (source_cluster_manager->GetNumStaticClusters() == 0) {
    return PairwiseMatches();
  }

  common::TicToc tictoc;
  SEGMENT_TIME_BEGIN(RunSemanticClusterMatcher);
  //----------------------------------------------------------
  // ComputeMiddleOutDescriptors
  //----------------------------------------------------------
  SEGMENT_TIME_BEGIN(ComputeMiddleOutDescriptors);
  const kindr::minimal::QuatTransformation &latest_pose = semantic_cluster_map_->latest_pose();
  const Point2d local_map_origin(
      {.x = latest_pose.getPosition().x(), .y = latest_pose.getPosition().y()});

  source_cluster_manager->ComputeMiddleOutDescriptors(params_.search_radius, local_map_origin,
                                                      params_.local_cluster_size);
  SEGMENT_TIME_END(ComputeMiddleOutDescriptors);

  if (!first_local_pose_) {
    first_local_pose_ = new Point2d();
    *first_local_pose_ = local_map_origin;
  }

  //----------------------------------------------------------
  // ClusterMatcher
  //----------------------------------------------------------
  SEGMENT_TIME_BEGIN(ClusterMatcher);
  PairwiseMatches candidate_matches;
  std::vector<int> target_indices;
  if (is_relocalization_successful_) {
    Eigen::Vector4f local_point;
    local_point << local_map_origin.x, local_map_origin.y, 0, 1;
    const Eigen::Vector4f target_point = map_lidar_transform_ * local_point;

    const Point2d target_map_origin({.x = target_point.x(), .y = target_point.y()});
    std::vector<float> k_distances;
    target_kdtree_->nearestKSearch(target_map_origin, params_.local_cluster_size, target_indices,
                                   k_distances);

    // TODO(silin): move to evalute class.
    // Record relocalization distance
    // if (!have_recorded_relozlization_distance_) {
    //   have_recorded_relozlization_distance_ = true;

    //   // Record match size.
    //   std::ofstream outfile(
    //       file::PathJoin(GetDataDirectoryPath(), "relocalization_distance.txt"),
    //       std::ios_base::app | std::ios_base::out);
    //   CHECK(outfile.good() && outfile.is_open());
    //   const double distance = std::sqrt((first_local_pose_->x - local_map_origin.x) *
    //                                         (first_local_pose_->x - local_map_origin.x) +
    //                                     (first_local_pose_->y - local_map_origin.y) *
    //                                         (first_local_pose_->y - local_map_origin.y));
    //   outfile << distance << std::endl;
    // }
  } else {
    target_indices.resize(target_clusters_manager_.num_clusters());
    std::iota(target_indices.begin(), target_indices.end(), 0);
  }

  for (const auto &source_index : source_cluster_manager->GetLocalClustersIndices()) {
    const auto &source_cluster = source_cluster_manager->at(source_index);
    for (const auto &target_index : target_indices) {
      const auto &target_cluster = target_clusters_manager_.at(target_index);

      if (cluster_matcher_->AreTwoClustersMatched(source_cluster, target_cluster)) {
        candidate_matches.emplace_back(source_cluster->id(), target_cluster->id(),
                                       source_cluster->centroid(), target_cluster->centroid());
      }
    }
  }
  SEGMENT_TIME_END(ClusterMatcher)

  LOG(INFO) << GREEN << "Candidate_matches matches size: " << candidate_matches.size() << COLOR_END;

  //----------------------------------------------------------
  // Recognize
  //----------------------------------------------------------
  SEGMENT_TIME_BEGIN(Recognize);
  recognizer_->recognize(candidate_matches);
  const std::vector<PairwiseMatches> &candidate_clusters = recognizer_->getCandidateClusters();

  if (candidate_clusters.empty()) {
    return PairwiseMatches();
  }
  const PairwiseMatches &matches = candidate_clusters.front();
  LOG(INFO) << GREEN << "Matches size: " << matches.size() << COLOR_END;
  SEGMENT_TIME_END(Recognize);

  //----------------------------------------------------------
  // ComputeTransform
  //----------------------------------------------------------
  SEGMENT_TIME_BEGIN(ComputeTransform);
  map_lidar_transform_ = recognizer_->getCandidateTransformations().front();
  LOG(INFO) << "Transform1: \n" << map_lidar_transform_;
  // ComputeTransformationfromMatches(matches, &map_lidar_transform_);
  // LOG(INFO) << "Transform2: " << map_lidar_transform_;
  SEGMENT_TIME_END(ComputeTransform);

  SEGMENT_TIME_END(RunSemanticClusterMatcher);
  is_relocalization_successful_ = true;

  // Record match size.
  // static std::ofstream outfile(file::PathJoin(GetDataDirectoryPath(), "tmp.txt"));
  // CHECK(outfile.good() && outfile.is_open());
  // outfile << matches.size() << std::endl;
  return matches;
}

void Relocalization::PublisherMatches(const PairwiseMatches &matches) {
  const Eigen::Transform<float, 3, Eigen::Affine> transform(map_lidar_transform_);

  PointPairs point_pairs(matches.size());
  for (size_t i = 0u; i < matches.size(); ++i) {
    Cluster::PointT source_cluster_centroid =
        pcl::transformPoint(matches[i].centroids().first, transform);
    Cluster::PointT target_cluster_centroid = matches[i].centroids().second;
    target_cluster_centroid.z -= params_.distance_to_lower_target_cloud_for_viz_m;
    point_pairs[i] = PointPair(source_cluster_centroid, target_cluster_centroid);
  }
  PublishLineSet(point_pairs, 0.3, Color(0.0, 1.0, 0.0));
}

void Relocalization::PublishLineSet(const PointPairs &point_pairs, const float line_scale,
                                    const Color &color) {
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = kMapFrameId;
  line_list.header.stamp = ros::Time();
  line_list.ns = "matches";
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.color.r = color.r;
  line_list.color.g = color.g;
  line_list.color.b = color.b;
  line_list.color.a = 1.0;
  line_list.scale.x = line_scale;
  for (size_t i = 0u; i < point_pairs.size(); ++i) {
    geometry_msgs::Point p;
    p.x = point_pairs[i].first.x;
    p.y = point_pairs[i].first.y;
    p.z = point_pairs[i].first.z;
    line_list.points.push_back(p);
    p.x = point_pairs[i].second.x;
    p.y = point_pairs[i].second.y;
    p.z = point_pairs[i].second.z;
    line_list.points.push_back(p);
  }
  matches_pub_.publish(line_list);
}

} // namespace long_term_relocalization

// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/20.
#pragma once

#include <array>

namespace long_term_relocalization {

// -------------------- common path --------------------
constexpr char kCurrentPackageName[] = "long_term_relocalization";
constexpr char kRelativeParamsFilePath[] = "/config/long_term_relocalization_params.yaml";
constexpr char kRelativeRs80AngleFilePath[] = "/config/rs80_laser_angles.csv";
constexpr char kRelativeDataDirPath[] = "/data";
constexpr char kRelativeMapDirPath[] = "/map";

// -------------------- ros topic --------------------
constexpr char kSourcePointCloudMapTopic[] = "/source_point_cloud_map";
constexpr char kSourceClustersCloudTopic[] = "/source_clusters_cloud";
constexpr char kSourceClustersCentroidsTopic[] = "/source_clusters_centroids";

constexpr char kTargetPointCloudMapTopic[] = "/target_point_cloud_map";
constexpr char kTargetClustersCloudTopic[] = "/target_clusters_cloud";
constexpr char kTargetClustersCentroidsTopic[] = "/target_clusters_centroids";

constexpr char kSaveMapCmdTopic[] = "/save_map_cmd";
constexpr char kMatchesTopic[] = "/matches";
constexpr char kLocalizationPathTopic[] = "/localization_path";

constexpr char kLidarPointsTopic[] = "/velodyne_points";
constexpr char kSemanticLidarPointsTopic[] = "/semantic_points";
constexpr char kLidarGroundPointsTopic[] = "/velodyne_ground_points";
constexpr char kLidarNonGroundPointsTopic[] = "/velodyne_non_ground_points";
constexpr char kLidarClusterPointsTopic[] = "/velodyne_cluster_points";
constexpr char kGroundOctoMapTopic[] = "/ground_octo_map";
constexpr char kGroundTrueOdomTopic[] = "/ground_true_odom";
constexpr char kImageTopic[] = "/image";
constexpr char kImuTopic[] = "/imu";
constexpr char kLaserOdometryTopic[] = "/lio_sam/mapping/odometry";
constexpr char kSemanticLidarPoseTopic[] = "/semantic_points_pose";

// -------------------- ros frame id --------------------
constexpr char kLidarPointsFrameId[] = "velodyne";
constexpr char kGroundTrueOdomFrameId[] = "ground_true_odom";
constexpr char kNormalGroundTrueOdomFrameId[] = "normal_ground_true_odom";
constexpr char kImageFrameId[] = "camera";
constexpr char kBodyFrameId[] = "body";
constexpr char kMapFrameId[] = "map";
constexpr char kImuFrameId[] = "imu";
constexpr char kLidarFrameId[] = "base_link";
constexpr char kOdomFrameId[] = "odom";

// -------------------- NCLT --------------------
constexpr char kImuFileName[] = "ms25.csv";
constexpr char kImuEulerFileName[] = "ms25_euler.csv";
constexpr double kVelodyneFrequencyInNclt = 5.0;
constexpr std::array<double, 6> kNormalGtToGtTransform = {0, 0, 0, M_PI, 0, 0}; // x y z r p y
constexpr char kRelativeNcltReaderSemanticParamsFilePath[] = "/config/nclt_semantic_params.yaml";
constexpr char kRelativeNcltReaderCamerasParamsFilePath[] = "/params/cam_params/K_cams.yaml";

// -------------------- NRSL --------------------
constexpr char kRelativeNrslReaderCamerasParamsFilePath[] =
    "/params/hit_sensors_params/nrsl_ladybug3_cams_intrinsic_params.yaml";
constexpr char kRelativeNrslLadybug3VirtualCamsIntrinsicFilePath[] =
    "/params/hit_sensors_params/nrsl_ladybug3_cams_params.yaml";

} // namespace long_term_relocalization

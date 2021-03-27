/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-19.

#pragma once

#include <string>
#include <vector>

#include <glog/logging.h>

#include <Eigen/Core>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>

#include "common/pcl_utils/pcl_types.h"
#include "common/pcl_utils/pcl_utils.h"
#include "long_term_relocalization/utils/params_types.h"

namespace long_term_relocalization {

struct Plane {
  Eigen::MatrixXf normal;
  double d = 0.;
};

/**
 * @brief Ground Removal based on Ground Plane Fitting(GPF)
 * @refer
 *   Fast Segmentation of 3D Point Clouds: A Paradigm on LiDAR Data for
 * Autonomous Vehicle Applications (ICRA, 2017)
 */
template <typename PointT> class GroundPlaneFitter {
  using PointCloud = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;

public:
  explicit GroundPlaneFitter(const GroundPlaneFitterParams &params) : params_(params) {
    static_assert(pcl::traits::has_xyz<PointT>::value,
                  "PointT must be a structure containing XYZ coordinates");
  }

  virtual ~GroundPlaneFitter() = default;

  void Process(const PointCloud &cloud_in, PointCloudPtr ground_cloud,
               PointCloudPtr nonground_cloud) {
    CHECK(ground_cloud != nullptr);
    CHECK(nonground_cloud != nullptr);
    ground_cloud->clear();
    nonground_cloud->clear();

    pcl::PointIndices::Ptr gnds_indices(new pcl::PointIndices);
    pcl::PointIndices::Ptr ngnds_indices(new pcl::PointIndices);
    Process(cloud_in, gnds_indices, ngnds_indices);
    pcl::copyPointCloud(cloud_in, *gnds_indices, *ground_cloud);
    pcl::copyPointCloud(cloud_in, *ngnds_indices, *nonground_cloud);
  }

  void Process(const PointCloud &cloud_in, pcl::PointIndices::Ptr ground_indices,
               pcl::PointIndices::Ptr nonground_indices) {
    CHECK(ground_indices != nullptr);
    CHECK(nonground_indices != nullptr);
    ground_indices->indices.clear();
    nonground_indices->indices.clear();

    if (cloud_in.empty()) {
      LOG(WARNING) << "Empty ground for ground fiiting, do nonthing.";
      return;
    }

    MainLoop(cloud_in, ground_indices, nonground_indices);
  }

private:
  /**
   * @brief Selection of initial seed points
   * @note
   *  Introduces the lowest point representative(LPR)
   *      ==> guarantees that noisy measurements will not affect the plane
   * estimation step
   * @input
   *  params_.gpf_num_lpr: number of  lowest point representative(LPR),
   * @param cloud_in
   * @param cloud_seeds
   */
  void ExtractInitialSeeds(const PointCloud &cloud_in, PointCloudPtr cloud_seeds) {
    std::vector<PointT> points(cloud_in.points.begin(), cloud_in.points.end());
    std::sort(points.begin(), points.end(), pcl_utils::PointZAxisCompare<PointT>);

    int cnt_lpr = 0;
    double height_average = 0.;
    // filter negative obstacles
    bool negative = true;
    for (size_t pt = 0u; pt < points.size() && cnt_lpr < params_.gpf_num_lpr; ++pt) {
      const double &h = points[pt].z;
      if (negative) {
        if (fabs(h + params_.gpf_sensor_height) > params_.gpf_th_lprs) {
          continue;
        } else {
          // because points are in "Incremental Order"
          negative = false;
        }
      }
      // collect from non-negative obstacles
      height_average += h;
      cnt_lpr++;
    }

    if (cnt_lpr > 0) {
      height_average /= cnt_lpr;
    } else {
      height_average = (-1.0) * params_.gpf_sensor_height;
    }

    // the points inside the height threshold are used as the initial seeds for
    // the plane model estimation
    (*cloud_seeds).clear();
    for (size_t pt = 0u; pt < points.size(); ++pt) {
      if (points[pt].z < height_average + params_.gpf_th_seeds) {
        (*cloud_seeds).points.push_back(points[pt]);
      }
    }
  }

  /**
   * @brief Estimate the ground plane(N^T X = -d) by SVD
   * @param cloud_seeds
   * @retval plane model estimation Plane
   */
  Plane EstimatePlane(const PointCloud &ground_cloud) {
    Plane model;

    // Create covariance matrix.
    // 1. calculate (x,y,z) mean
    float mean_x = 0., mean_y = 0., mean_z = 0.;
    for (size_t pt = 0u; pt < ground_cloud.points.size(); ++pt) {
      mean_x += ground_cloud.points[pt].x;
      mean_y += ground_cloud.points[pt].y;
      mean_z += ground_cloud.points[pt].z;
    }
    if (ground_cloud.points.size()) {
      mean_x /= ground_cloud.points.size();
      mean_y /= ground_cloud.points.size();
      mean_z /= ground_cloud.points.size();
    }
    // 2. calculate covariance
    // cov(x,x), cov(y,y), cov(z,z)
    // cov(x,y), cov(x,z), cov(y,z)
    float cov_xx = 0., cov_yy = 0., cov_zz = 0.;
    float cov_xy = 0., cov_xz = 0., cov_yz = 0.;
    for (int i = 0; i < ground_cloud.points.size(); i++) {
      cov_xx += (ground_cloud.points[i].x - mean_x) * (ground_cloud.points[i].x - mean_x);
      cov_xy += (ground_cloud.points[i].x - mean_x) * (ground_cloud.points[i].y - mean_y);
      cov_xz += (ground_cloud.points[i].x - mean_x) * (ground_cloud.points[i].z - mean_z);
      cov_yy += (ground_cloud.points[i].y - mean_y) * (ground_cloud.points[i].y - mean_y);
      cov_yz += (ground_cloud.points[i].y - mean_y) * (ground_cloud.points[i].z - mean_z);
      cov_zz += (ground_cloud.points[i].z - mean_z) * (ground_cloud.points[i].z - mean_z);
    }
    // 3. setup covariance matrix Cov
    Eigen::MatrixXf Cov(3, 3);
    Cov << cov_xx, cov_xy, cov_xz, cov_xy, cov_yy, cov_yz, cov_xz, cov_yz, cov_zz;
    Cov /= ground_cloud.points.size();

    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> SVD(Cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    model.normal = (SVD.matrixU().col(2));
    // d is directly computed substituting x with s^ which is a good
    // representative for the points belonging to the plane
    Eigen::MatrixXf mean_seeds(3, 1);
    mean_seeds << mean_x, mean_y, mean_z;
    // according to normal^T*[x,y,z]^T = -d
    model.d = -(model.normal.transpose() * mean_seeds)(0, 0);

    // ROS_WARN_STREAM("Model: " << model.normal << " " << model.d);

    return model;
  }

  /**
   * @brief GPF main loop for one segemnt of the point cloud
   * @param cloud_in
   * @param cloud_gnds
   * @param cloud_ngnds
   */
  void MainLoop(const PointCloud &cloud_in, pcl::PointIndices::Ptr gnds_indices,
                pcl::PointIndices::Ptr ngnds_indices) {
    PointCloudPtr cloud_seeds(new PointCloud);
    ExtractInitialSeeds(cloud_in, cloud_seeds);

    PointCloudPtr cloud_gnds(new PointCloud(*cloud_seeds));

    for (size_t iter = 0u; iter < params_.gpf_num_iter; ++iter) {
      Plane model = EstimatePlane(*cloud_gnds);
      // clear
      cloud_gnds->clear();
      gnds_indices->indices.clear();
      ngnds_indices->indices.clear();
      // pointcloud to matrix
      Eigen::MatrixXf cloud_matrix(cloud_in.points.size(), 3);
      size_t pi = 0u;
      for (const auto &p : cloud_in.points) {
        cloud_matrix.row(pi++) << p.x, p.y, p.z;
      }
      // distance to extimated ground plane model (N^T X)^T = (X^T N)
      Eigen::VectorXf dists = cloud_matrix * model.normal;
      // threshold filter: N^T xi + d = dist < th_dist ==> N^T xi < th_dist -
      // d
      double th_dist = params_.gpf_th_gnds - model.d;
      for (size_t pt = 0u; pt < dists.rows(); ++pt) {
        if (dists[pt] < th_dist) {
          gnds_indices->indices.push_back(pt);
        } else {
          ngnds_indices->indices.push_back(pt);
        }
      }
      // extract ground points
      pcl::copyPointCloud(cloud_in, *gnds_indices, *cloud_gnds);
    }
  }

private:
  GroundPlaneFitterParams params_;
};

} // namespace long_term_relocalization

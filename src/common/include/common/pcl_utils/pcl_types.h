// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-1.

#pragma once
#include <stdint.h>

#include <Eigen/Dense>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>

struct __PointXYZIR {
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  std::uint16_t ring;             ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(__PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(std::uint16_t,
                                                                                     ring, ring))

struct __PointXYZIRL {
  PCL_ADD_POINT4D;     // quad-word XYZ
  float intensity;     ///< laser intensity reading
  std::uint16_t ring;  ///< laser ring number
  std::uint16_t label; ///< point semantic label

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(__PointXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(
                                      std::uint16_t, ring, ring)(std::uint16_t, label, label))

struct __PointXYZIRT {
  PCL_ADD_POINT4D;    // quad-word XYZ
  float intensity;    ///< laser intensity reading
  std::uint16_t ring; ///< laser ring number
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(__PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is
 * time stamp)
 */
struct __PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(__PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(float, roll,
                                                                                     roll)(
                                      float, pitch, pitch)(float, yaw, yaw)(double, time, time))

namespace pcl_utils {

typedef __PointXYZIR PointIR;
typedef __PointXYZIRL PointIRL;
typedef __PointXYZIRT PointIRT;
typedef __PointXYZIRPYT PointXYZIRPYT;

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZI PointI;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZRGBL PointRGBL;

typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<PointI> PointICloud;
typedef pcl::PointCloud<PointIR> PointIRCloud;
typedef pcl::PointCloud<PointIRT> PointIRTCloud;
typedef pcl::PointCloud<PointIRL> PointIRLCloud;
typedef pcl::PointCloud<PointRGB> PointRGBCloud;
typedef pcl::PointCloud<PointRGBL> PointRGBLCloud;
typedef pcl::PointCloud<PointXYZIRPYT> PointIRPYTCloud;

} // namespace pcl_utils

// Copyright (c) ETH asl. All rights reserved.
// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/7.

#pragma once

#include <pcl/common/transforms.h>

#include "mapping/dynamic_voxel_grid.h"
#include "mapping/kdtree_points_neighbors_provider.h"
#include "utils/common/pcl_types.h"
#include "utils/transform/timestamped_transform.h"

namespace long_term_relocalization {
namespace mapping {

/// \brief Parameters of the local map.
struct LocalMapParams {
  /// \brief Size of a voxel in the grid.
  float voxel_size_m = .0;
  /// \brief Minimum number of points that a voxel must contain in order to be
  /// considered active.
  int min_points_per_voxel = 0;
  /// \brief Radius of the local map.
  float radius_m = .0;
  /// \brief Minimum vertical distance between a point and the robot.
  float min_vertical_distance_m = .0;
  /// \brief Maximum vertical distance between a point and the robot.
  float max_vertical_distance_m = .0;
};

/// \brief Manages the local point cloud of a robot. Provides methods for inserting, filtering and
/// segmenting points.
/// \remark The class is \e not thread-safe. Concurrent access to the class results in undefined
/// behavior.
template <typename InputPointT, typename ClusteredPointT> class LocalMap {
public:
  typedef DynamicVoxelGrid<InputPointT, ClusteredPointT> VoxelGrid;
  typedef typename VoxelGrid::InputCloud InputCloud;
  typedef typename VoxelGrid::VoxelCloud ClusteredCloud;

  /// \brief Initializes a new instance of the LocalMap class.
  /// \param params The parameters of the local map.
  /// \param normal_estimator Pointer to an object that can be used for estimating the normals. If
  /// null, normals will not be estimated.
  LocalMap(const LocalMapParams &params //, std::unique_ptr<NormalEstimator> normal_estimator
  );

  /// \brief Move constructor for the LocalMap class.
  /// \param other The object to be moved in this instance.
  LocalMap(LocalMap &&other)
      : voxel_grid_(std::move(other.voxel_grid_)), radius_squared_m2_(other.radius_squared_m2_),
        min_vertical_distance_m_(other.min_vertical_distance_m_),
        max_vertical_distance_m_(other.max_vertical_distance_m_),
        points_neighbors_provider_(std::move(other.points_neighbors_provider_))
        // ,normal_estimator_(std::move(other.normal_estimator_))
        {};

  /// \brief Update the pose of the robot and add new points to the local map.
  /// \param new_cloud Vector of point clouds to be added.
  /// \param pose The new pose of the robot.
  void updatePoseAndAddPoints(const InputCloud &new_cloud /*in lidar frame*/,
                              const transform::Rigid3d &pose /*in world frame*/);

  /// \brief Apply a pose transformation to the points contained in the local map.
  /// \remark Multiple transformations are cumulative.
  /// \param transformation The transformation to be applied to the local map.
  void transform(const kindr::minimal::QuatTransformationTemplate<float> &transformation);

  /// \brief Clears the local map, removing all the points it contains.
  void clear();

  /// \brief Gets a filtered view of the points contained in the point cloud.
  /// \remark Modifying the X, Y, Z components of the points in the returned cloud results in
  /// undefined behavior.
  /// \return Reference to the clustered cloud.
  ClusteredCloud &getFilteredPoints() const { return voxel_grid_.getActiveCentroids(); }

  /// \brief Gets a filtered view of the points contained in the point cloud.
  /// \remark Modifying the X, Y, Z components of the points in the returned cloud results in
  /// undefined behavior.
  /// \return Pointer to the clustered cloud.
  typename ClusteredCloud::ConstPtr getFilteredPointsPtr() const {
    return typename ClusteredCloud::ConstPtr(&voxel_grid_.getActiveCentroids(),
                                             [](ClusteredCloud const *ptr) {});
  }

  /// \brief Gets the normals of the points of the local map.
  /// \remark The returned value is valid only if the local map has been constructed with the
  /// \c estimate_normals option set to true. Otherwise, the normal cloud is empty.
  /// \return Normals of the points of the map.

  // const PointNormals &getNormals() const {
  //   if (normal_estimator_ != nullptr)
  //     return normal_estimator_->getNormals();
  //   else
  //     return empty_normals_cloud_;
  // }

  /// \brief Gets an object that can be used for nearest neighbors queries on the points of the
  /// local map.
  /// \returns The points neighbors provider object.
  inline KdTreePointsNeighborsProvider<ClusteredPointT> &getPointsNeighborsProvider() {
    return *points_neighbors_provider_;
  }

  /// \brief Gets a reference to the current mapping from clusters to segment IDs. Cluster \c i has
  /// segment ID <tt>getClusterToSegmentIdMapping()[i]</tt>.
  /// \return Reference to the mapping between clusters and segment IDs.
  std::vector<common::Id> &getClusterToSegmentIdMapping() { return segment_ids_; }

  /// \brief Gets the indices of the normals that have been modified since the last update.
  /// \returns Indices of the modified normals.
  std::vector<bool> getIsNormalModifiedSinceLastUpdate() {
    return is_normal_modified_since_last_update_;
  }

private:
  std::vector<bool> updatePose(const transform::Rigid3d &pose);
  std::vector<int> addPointsAndGetCreatedVoxels(const InputCloud &new_cloud);
  std::vector<int> buildPointsMapping(const std::vector<bool> &is_point_removed,
                                      const std::vector<int> &new_points_indices);

  VoxelGrid voxel_grid_; // part I

  const float radius_squared_m2_;
  const float min_vertical_distance_m_;
  const float max_vertical_distance_m_;

  std::unique_ptr<KdTreePointsNeighborsProvider<ClusteredPointT>> points_neighbors_provider_;
  // std::unique_ptr<NormalEstimator> normal_estimator_; // part II
  // PointNormals empty_normals_cloud_;

  // Variables needed for working with incremental updates.
  std::vector<common::Id> segment_ids_;
  std::vector<bool> is_normal_modified_since_last_update_;
}; // class LocalMap

//=================================================================================================
//    LocalMap public methods implementation
//=================================================================================================

template <typename InputPointT, typename ClusteredPointT>
LocalMap<InputPointT, ClusteredPointT>::LocalMap(
    const LocalMapParams &params
    //, std::unique_ptr<NormalEstimator> normal_estimator
    )
    : voxel_grid_(params.voxel_size_m, params.min_points_per_voxel),
      radius_squared_m2_(pow(params.radius_m, 2.0)),
      min_vertical_distance_m_(params.min_vertical_distance_m),
      max_vertical_distance_m_(params.max_vertical_distance_m)
// ,normal_estimator_(std::move(normal_estimator))
{
  points_neighbors_provider_ = std::unique_ptr<KdTreePointsNeighborsProvider<ClusteredPointT>>(
      new KdTreePointsNeighborsProvider<ClusteredPointT>());
}

template <typename InputPointT, typename ClusteredPointT>
void LocalMap<InputPointT, ClusteredPointT>::updatePoseAndAddPoints(
    const InputCloud &new_cloud, const transform::Rigid3d &pose) {
  // BENCHMARK_BLOCK("SM.UpdateLocalMap");

  InputCloud cloud_out; // world frame
  pcl::transformPointCloud(new_cloud, cloud_out, transform::Rigid3dToMatrix4d(pose));

  // 返回需要移除的active状态的voxel下active_centroids_中的index
  std::vector<bool> is_point_removed = updatePose(pose);
  // 将新来的点云加入local map，更新voxel_和active_centroids_，inactive_centroids_
  // 同时得到那些由inactive变为active的voxel在active_centroids_下的index
  std::vector<int> created_points_indices = addPointsAndGetCreatedVoxels(cloud_out);
  std::vector<int> points_mapping = buildPointsMapping(is_point_removed, created_points_indices);

  // Update the points neighbors provider.
  // BENCHMARK_START("SM.UpdateLocalMap.UpdatePointsNeighborsProvider");
  getPointsNeighborsProvider().update(getFilteredPointsPtr(), {});
  // BENCHMARK_STOP("SM.UpdateLocalMap.UpdatePointsNeighborsProvider");

  // If required, update the normals.
  // if (normal_estimator_ != nullptr) {
  //   BENCHMARK_BLOCK("SM.UpdateLocalMap.EstimateNormals");
  //   is_normal_modified_since_last_update_ = normal_estimator_->updateNormals(
  //       getFilteredPoints(), points_mapping, created_points_indices,
  //       getPointsNeighborsProvider());
  // } else {
  //   is_normal_modified_since_last_update_ = std::vector<bool>(getFilteredPoints().size(), false);
  // }
}

template <typename InputPointT, typename ClusteredPointT>
std::vector<bool>
LocalMap<InputPointT, ClusteredPointT>::updatePose(const transform::Rigid3d &pose) {
  // BENCHMARK_BLOCK("SM.UpdateLocalMap.UpdatePose");

  pcl::PointXYZ position;
  position.x = pose.translation().x();
  position.y = pose.translation().y();
  position.z = pose.translation().z();

  // Remove points according to a cylindrical filter predicate.
  std::vector<bool> is_point_removed = voxel_grid_.removeIf([&](const ClusteredPointT &p) {
    float distance_xy_squared = pow(p.x - position.x, 2.0) + pow(p.y - position.y, 2.0);
    bool remove = distance_xy_squared > radius_squared_m2_ ||
                  p.z - position.z < min_vertical_distance_m_ ||
                  p.z - position.z > max_vertical_distance_m_;
    // TODO: Once we start supporting multiple segmenters working on the same cloud, we will need
    // one \c segment_ids_ vector per segmenter.
    // if (remove && p.ed_cluster_id != 0u)
    //   segment_ids_[p.ed_cluster_id] = common::kInvId;
    // if (remove && p.sc_cluster_id != 0u)
    //   segment_ids_[p.sc_cluster_id] = common::kInvId;
    return remove;
  });

  return is_point_removed;
}

template <typename InputPointT, typename ClusteredPointT>
std::vector<int>
LocalMap<InputPointT, ClusteredPointT>::addPointsAndGetCreatedVoxels(const InputCloud &new_cloud) {
  // BENCHMARK_BLOCK("SM.UpdateLocalMap.AddNewPoints");
  std::vector<int> created_points_indices = voxel_grid_.insert(new_cloud);

  // Record local map metrics.
  // BENCHMARK_RECORD_VALUE("SM.UpdateLocalMap.InsertedPoints", merged_cloud.size());
  // BENCHMARK_RECORD_VALUE("SM.UpdateLocalMap.CreatedVoxels", created_points_indices.size());
  // BENCHMARK_RECORD_VALUE("SM.UpdateLocalMap.ActiveVoxels", getFilteredPoints().size());
  // BENCHMARK_RECORD_VALUE("SM.UpdateLocalMap.InactiveVoxels",
  //                        voxel_grid_.getInactiveCentroids().size());

  return created_points_indices;
}

template <typename InputPointT, typename ClusteredPointT>
std::vector<int> LocalMap<InputPointT, ClusteredPointT>::buildPointsMapping(
    const std::vector<bool> &is_point_removed, const std::vector<int> &new_points_indices) {
  // BENCHMARK_BLOCK("SM.UpdateLocalMap.BuildPointsMapping");

  // Build a mapping from index in the old point cloud to index in the new point cloud.
  size_t new_point_index = 0u;
  size_t next_inserted_point_index = 0u;
  std::vector<int> mapping(is_point_removed.size());

  for (size_t old_point_index = 0u; old_point_index < is_point_removed.size(); ++old_point_index) {
    if (is_point_removed[old_point_index]) {
      // Mark point as removed.
      mapping[old_point_index] = -1;
    } else //不需要移除
    {
      while (next_inserted_point_index < new_points_indices.size() &&
             new_points_indices[next_inserted_point_index] == new_point_index) {
        // Skip any inserted point, they don't belong to the mapping.
        ++new_point_index;
        ++next_inserted_point_index;
      }
      mapping[old_point_index] = new_point_index++;
    }
  }

  return mapping;
}

template <typename InputPointT, typename ClusteredPointT>
void LocalMap<InputPointT, ClusteredPointT>::transform(
    const kindr::minimal::QuatTransformationTemplate<float> &transformation) {
  // BENCHMARK_BLOCK("SM.TransformLocalMap");
  voxel_grid_.transform(transformation);

  // if (normal_estimator_ != nullptr) {
  //   BENCHMARK_BLOCK("SM.TransformLocalMap.TransformNormals");
  //   normal_estimator_->notifyPointsTransformed(transformation);
  // }
}

template <typename InputPointT, typename ClusteredPointT>
void LocalMap<InputPointT, ClusteredPointT>::clear() {
  voxel_grid_.clear();
  // if (normal_estimator_ != nullptr)
  //   normal_estimator_->clear();
}

} // namespace mapping
} // namespace long_term_relocalization

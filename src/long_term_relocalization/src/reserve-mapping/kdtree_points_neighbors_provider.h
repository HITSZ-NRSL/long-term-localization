// Copyright (c) ETH asl. All rights reserved.
// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/15.

#pragma once

#include <vector>

#include <pcl/search/kdtree.h>

#include "common/pcl_utils/pcl_types.h"

namespace long_term_relocalization {
namespace mapping {

/// \brief Indices of the neighbors of a point.
typedef std::vector<int> PointNeighbors;

/// \brief Provides point neighborhood information of a point cloud by building a k-d tree and
/// querying the neighbors of each point.
template <typename PointT> class KdTreePointsNeighborsProvider {
public:
  typedef pcl::PointCloud<PointT> PointCloud;

  static_assert(pcl::traits::has_xyz<PointT>::value,
                "KdTreePointsNeighborsProvider requires PointT to contain XYZ coordinates.");

  /// \brief Initializes a new instance of the KdTreePointsNeighborsProvider class.
  KdTreePointsNeighborsProvider() : kd_tree_(), point_cloud_(nullptr) {}

  /// \brief Update the points neighborhood provider.
  /// \param point_cloud The new point cloud.
  /// \param points_mapping Mapping from the points stored in the current cloud to the points of
  /// the new point cloud. Point \c i is moved to position \c points_mapping[i]. Values smaller
  /// than 0 indicate that the point has been removed.
  /// \remarks point_cloud must remain a valid object during all the successive calls to
  /// getNeighborsOf()
  void update(const typename pcl::PointCloud<PointT>::ConstPtr point_cloud,
              const std::vector<int64_t> &points_mapping = {});

  /// \brief Gets the indexes of the neighbors of the point with the specified index.
  /// \param point_index Index of the query point.
  /// \param search_radius The radius of the searched neighborhood.
  /// \return Vector containing the indices of the neighbor points.
  const PointNeighbors getNeighborsOf(size_t point_index, float search_radius);

  /// \brief Returns the underlying PCL search object.
  /// \remarks This function is present only for compatibility with the old segmenters and
  /// should not be used in new code.
  /// \returns Pointer to the PCL search object. If the provider doesn't use PCL search objects
  /// this function returns null.
  typename pcl::search::Search<PointT>::Ptr getPclSearchObject() {
    return typename pcl::search::KdTree<PointT>::Ptr(&kd_tree_,
                                                     [](pcl::search::KdTree<PointT> *ptr) {});
  }

private:
  // The tree used for neighbor searches.
  pcl::search::KdTree<PointT> kd_tree_;
  typename pcl::PointCloud<PointT>::ConstPtr point_cloud_;
}; // class KdTreePointsNeighborsProvider

//=================================================================================================
//    KdTreePointsNeighborsProvider public methods implementation
//=================================================================================================

template <typename PointT>
void KdTreePointsNeighborsProvider<PointT>::update(
    const typename pcl::PointCloud<PointT>::ConstPtr point_cloud,
    const std::vector<int64_t> &points_mapping) {
  // Build k-d tree
  point_cloud_ = point_cloud;
  kd_tree_.setInputCloud(point_cloud_);
}

template <typename PointT>
const PointNeighbors
KdTreePointsNeighborsProvider<PointT>::getNeighborsOf(const size_t point_index,
                                                      const float search_radius) {
  CHECK(point_cloud_ != nullptr);

  // Get the neighbors from the kd-tree.
  std::vector<int> neighbors_indices;
  std::vector<float> neighbors_distances;
  kd_tree_.radiusSearch((*point_cloud_)[point_index], search_radius, neighbors_indices,
                        neighbors_distances);

  return neighbors_indices;
}

} // namespace mapping
} // namespace long_term_relocalization
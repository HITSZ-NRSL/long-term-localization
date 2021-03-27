// Copyright (c) ETH asl. All rights reserved.
// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/15.

#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <vector>

#include <glog/logging.h>

#include <kindr/minimal/quat-transformation.h>

#include <glog/logging.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/pcl_utils/pcl_types.h"

namespace long_term_relocalization {
namespace mapping {

/// \brief A grid of cubic volume cells.
/// 向DVG中插入的点被放到指定的体素内。网格提供了点的下采样，并支持根据predicate规则移除体素。
/// 网格区分active体素（voxel中点超过一定数量）和inactive体素。这个是为了降低noise
/// \remark The class is \e not thread-safe. Concurrent access to the class results in undefined
/// behavior.
template <typename InputPointT,       // 点云点
          typename VoxelPointT,       // 珊格点
          typename IndexT = uint64_t, // 珊格点index
          uint8_t bits_x = 20,        // 珊格尺寸2^bits_x*2^bits_y*2^bits_z
          uint8_t bits_y = 20, uint8_t bits_z = 20>
class DynamicVoxelGrid {
public:
  typedef typename pcl::PointCloud<InputPointT> InputCloud;
  typedef typename pcl::PointCloud<VoxelPointT> VoxelCloud;

  static_assert(std::is_integral<IndexT>::value && std::is_unsigned<IndexT>::value,
                "IndexT must be an unsigned integral type");
  static_assert(bits_x + bits_y + bits_z <= sizeof(IndexT) * 8,
                "The number of bits required per dimension is bigger than the size of IndexT");
  static_assert(bits_x > 0 && bits_y > 0 && bits_z > 0,
                "The index requires at least one bit per dimension");
  static_assert(pcl::traits::has_xyz<InputPointT>::value,
                "InputPointT must be a structure containing XYZ coordinates");
  static_assert(pcl::traits::has_xyz<VoxelPointT>::value,
                "VoxelPointT must be a structure containing XYZ coordinates");

  /// \brief: 初始化一个DynamicVoxelGrid
  /// \param resolution: voxel分辨率，长度，单位：m
  /// \param min_points_per_voxel: 每个体素的最小点：体素必须包含的最小点数，才能被视为活动点。
  /// \param origin: 体素中心
  DynamicVoxelGrid(const float resolution, const int min_points_per_voxel,
                   const InputPointT &origin = InputPointT())
      : resolution_(resolution), min_points_per_voxel_(min_points_per_voxel),
        origin_(origin), // 原点
        grid_size_(      // 珊格真实尺寸，单位m
            resolution * static_cast<float>(n_voxels_x),
            resolution * static_cast<float>(n_voxels_y),
            resolution * static_cast<float>(n_voxels_z)),
        origin_offset_(origin.getVector3fMap()),              // 世界坐标系原点
        indexing_offset_(grid_size_ / 2.0f - origin_offset_), // 体素坐标系原点是珊格中心
        world_to_grid_(1.0f / resolution), // 世界坐标系坐标转珊格坐标系index
        min_corner_(origin_offset_ - grid_size_ / 2.0f), // 体素左下角
        max_corner_(origin_offset_ + grid_size_ / 2.0f), // 体素右上角
        active_centroids_(new VoxelCloud()),             // 激活状态的点云中心
        inactive_centroids_(new VoxelCloud()),           // 非激活状态的点云中心
        pose_transformation_(), indexing_transformation_() {
    // Validate inputs.
    CHECK_GT(resolution, 0.0f);
    CHECK_GE(min_points_per_voxel, 1);
  }

  /// \brief Move constructor for the DynamicVoxelGrid class.
  /// \param other Object that has to be moved into the new instance.
  DynamicVoxelGrid(DynamicVoxelGrid &&other)
      : resolution_(other.resolution_), min_points_per_voxel_(other.min_points_per_voxel_),
        origin_(std::move(other.origin_)), grid_size_(std::move(other.grid_size_)),
        origin_offset_(std::move(other.origin_offset_)),
        indexing_offset_(std::move(other.indexing_offset_)), world_to_grid_(other.world_to_grid_),
        min_corner_(std::move(other.min_corner_)), max_corner_(std::move(other.max_corner_)),
        active_centroids_(std::move(other.active_centroids_)),
        inactive_centroids_(std::move(other.inactive_centroids_)),
        voxels_(std::move(other.voxels_)),
        pose_transformation_(std::move(other.pose_transformation_)),
        indexing_transformation_(std::move(other.indexing_transformation_)) {}

  /// \brief 向DVG中插入点云
  /// 插入新点会更新点的X、Y、Z坐标，但可能会留下任何额外的未触及区域
  /// \remark 插入使对centroids的任何引用无效。
  /// \param new_cloud 待插入点云
  /// \returns 返回插入DVG之后，状态为active的voxel的centroid的indice
  std::vector<int> insert(const InputCloud &new_cloud);

  /// \brief Result of a removal operation.
  /// \remark Enabled only for predicates of the form: <tt>bool p(const VoxelPointT&)</tt>
  template <typename Func>
  using RemovalResult = typename std::enable_if<
      std::is_convertible<Func, std::function<bool(const VoxelPointT &)>>::value,
      std::vector<bool>>::type;

  /// \brief Removes from the grid a set of voxels satisfying the given predicate.
  /// \remark Removal invalidates any reference to the centroids.
  /// \returns Vector indicating, for each active voxel index, if the centroid has been removed or
  /// not.
  template <typename Func> RemovalResult<Func> removeIf(Func predicate);

  /// \brief Compute the index of the voxel containing the specified point.
  template <typename PointXYZ_> IndexT getIndexOf(const PointXYZ_ &point) const;

  /// \brief Apply a pose transformation to the voxel grid.
  /// \remark Multiple transformations are cumulative.
  /// \param transformation The transformation to be applied to the grid.
  void transform(const kindr::minimal::QuatTransformationTemplate<float> &transformation);

  /// \brief Clears the dynamic voxel grid, removing all the points it contains and resetting the
  /// transformations.
  void clear();

  /// \brief Returns a reference to the centroids of the active voxels.
  /// \remark Modifying the X, Y, Z components of the points in the returned cloud results in
  /// undefined behavior.
  /// \returns The centroids of the active voxels.
  inline VoxelCloud &getActiveCentroids() const { return *active_centroids_; }

  /// \brief Returns a reference to the centroids of the inactive voxels.
  /// \remark Modifying the X, Y, Z components of the points in the returned cloud results in
  /// undefined behavior.
  /// \returns The centroids of the inactive voxels.
  inline VoxelCloud &getInactiveCentroids() const { return *inactive_centroids_; }

  /// \brief Dump informations about the voxels contained in the grid.
  void dumpVoxels() const;

private:
  // A point with its voxel index.
  // 索引点，存储输入点，点所属的voxel在voxel map中的index
  struct IndexedPoint_ {
    IndexedPoint_(const InputPointT &point, const IndexT &voxel_index)
        : point(point), voxel_index(voxel_index) {}

    InputPointT point;  // pcl::PointXYZ
    IndexT voxel_index; // voxel在voxel map中的index
  };
  typedef std::vector<IndexedPoint_> IndexedPoints_; //索引点云

  // A voxel in the grid
  // 每一个体素存储，体素点云中心，点云点个数，该体素在voxel地图中的index
  struct Voxel_ {
    Voxel_() : centroid(nullptr), index(0), num_points(0) {}

    Voxel_(VoxelPointT *centroid, const IndexT &index, const uint32_t num_points)
        : centroid(centroid), index(index), num_points(num_points) {}

    Voxel_(const Voxel_ &other)
        : centroid(other.centroid), index(other.index), num_points(other.num_points) {}

    VoxelPointT *centroid;
    IndexT index;
    uint32_t num_points;
  };

  // The data necessary to construct a voxel.
  struct VoxelData_ {
    Voxel_ *old_voxel;
    typename IndexedPoints_::iterator points_begin;
    typename IndexedPoints_::iterator points_end;
  };

  // Compute the voxel indices of a point cloud and sort the points in increasing voxel index order.
  IndexedPoints_ indexAndSortPoints_(const InputCloud &points) const;

  // Create a voxel staring from the data about the point it contains and insert it in the voxels
  // and centroids vectors. Returns true if the new points inserted triggered the voxel.
  bool createVoxel_(const IndexT index, const VoxelData_ &data, std::vector<Voxel_> &new_voxels,
                    VoxelCloud &new_active_centroids, VoxelCloud &new_inactive_centroids);

  // Removes the centroids at the specified pointers. The pointers must be sorted in increasing
  // order.
  std::vector<bool> removeCentroids_(VoxelCloud &target_cloud,
                                     std::vector<VoxelPointT *> to_remove);

  // The centroids of the voxels containing enough points.
  std::unique_ptr<VoxelCloud> active_centroids_;
  std::unique_ptr<VoxelCloud> inactive_centroids_;

  // The voxels in the point cloud.
  std::vector<Voxel_> voxels_;

  // Properties of the grid.
  const float resolution_;
  const int min_points_per_voxel_;
  const InputPointT origin_;

  // Size of the voxel grid.
  static constexpr IndexT n_voxels_x = (IndexT(1) << bits_x);
  static constexpr IndexT n_voxels_y = (IndexT(1) << bits_y);
  static constexpr IndexT n_voxels_z = (IndexT(1) << bits_z);

  // Variables needed for conversion from world coordinates to voxel index.
  const Eigen::Vector3f grid_size_;
  const Eigen::Vector3f origin_offset_;
  const Eigen::Vector3f indexing_offset_;
  const Eigen::Vector3f min_corner_;
  const Eigen::Vector3f max_corner_;
  float world_to_grid_;
  kindr::minimal::QuatTransformationTemplate<float> pose_transformation_;
  kindr::minimal::QuatTransformationTemplate<float> indexing_transformation_;
}; // class DynamicVoxelGrid

//=================================================================================================
//    DynamicVoxelGrid public methods implementation
//=================================================================================================

// Short name macros for Dynamic Voxel Grid (DVG) template declaration and specification.
#define _DVG_TEMPLATE_DECL_                                                                        \
  typename InputPointT, typename VoxelPointT, typename IndexT, uint8_t bits_x, uint8_t bits_y,     \
      uint8_t bits_z
#define _DVG_TEMPLATE_SPEC_ InputPointT, VoxelPointT, IndexT, bits_x, bits_y, bits_z

// 移除局部地图（圆柱形）范围之外的旧点
// 更新 voxels_, inactive_centroids_, active_centroids_
// return: 返回那些需要移除的active的voxel中active_centroids_的index
template <_DVG_TEMPLATE_DECL_>
template <typename Func>
inline DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::RemovalResult<Func>
DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::removeIf(Func predicate) {
  // Setup iterators
  auto v_read = voxels_.begin();
  const auto v_end = voxels_.end();

  // Returns a reference to the point cloud containing the centroid of the
  // specified voxel.
  std::vector<VoxelPointT *> active_centroids_to_remove;
  std::vector<VoxelPointT *> inactive_centroids_to_remove;
  auto get_centroids_container_for = [&](const Voxel_ &voxel) -> std::vector<VoxelPointT *> & {
    if (voxel.num_points >= min_points_per_voxel_) {
      return active_centroids_to_remove;
    } else {
      return inactive_centroids_to_remove;
    }
  };

  // Remove the voxels and collect the pointers of the centroids that must be
  // removed.
  while (v_read != v_end && !predicate(*(v_read->centroid)))
    ++v_read;

  if (v_read == v_end)
    return std::vector<bool>(active_centroids_->size(), false);
  auto v_write = v_read;
  get_centroids_container_for(*v_read).push_back(v_read->centroid);
  ++v_read;

  for (; v_read != v_end; ++v_read) {
    if (!predicate(*(v_read->centroid))) {
      // Copy the centroid, updating the pointer from the voxel.
      *v_write = *v_read;
      v_write->centroid -= get_centroids_container_for(*v_read).size();
      ++v_write;
    } else {
      // Keep track of the voxels that need to be deleted.
      get_centroids_container_for(*v_read).push_back(v_read->centroid);
    }
  }

  voxels_.erase(v_write, v_end);

  // Actually remove centroids
  removeCentroids_(*inactive_centroids_, inactive_centroids_to_remove);
  return removeCentroids_(*active_centroids_, active_centroids_to_remove);
}

/// \brief:  Insert Point Cloud
/// \param new_cloud: 待插入点云
/// \return: 点云中每个点在DVG中的voxel indice vector
template <_DVG_TEMPLATE_DECL_>
std::vector<int> DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::insert(const InputCloud &new_cloud) {
  // 新点云创建对应点个数的下标数组
  std::vector<int> created_voxel_indices;
  if (new_cloud.empty())
    return created_voxel_indices;

  // 内存预留
  created_voxel_indices.reserve(new_cloud.size());

  // 点云排序，得到排序后的索引点云，此时给点云中的每个点都找到了应该属于的voxel index
  IndexedPoints_ new_points = indexAndSortPoints_(new_cloud);

  // Create containers and reserve space to prevent reallocation
  std::vector<Voxel_>
      new_voxels; // 加入点云之后的新的体素，按照最大需求进行内存预留，假设每个点都是一个单独的voxel
  new_voxels.reserve(voxels_.size() + new_cloud.size());
  std::unique_ptr<VoxelCloud> new_active_centroids(new VoxelCloud()); // 激活状态点中心，内存预留
  new_active_centroids->reserve(active_centroids_->size() + new_cloud.size());
  std::unique_ptr<VoxelCloud> new_inactive_centroids(new VoxelCloud()); //非激活状态点中心，内存预留
  new_inactive_centroids->reserve(inactive_centroids_->size() + new_cloud.size());

  // 设置索引点云new_points和voxels的迭代器
  auto p_it = new_points.begin();
  auto v_it = voxels_.begin();
  const auto p_end = new_points.end();
  const auto v_end = voxels_.end();

  // Merge points updating the affected voxels.
  // 终止条件是点云和voxel都遍历完毕
  while (!(p_it == p_end && v_it == v_end)) {
    // 遍历过程中voxel data和voxel index记录
    VoxelData_ voxel_data = {nullptr, p_it,
                             p_it}; //存放voxel_data起始点在索引点云中的位置points_begin
    IndexT voxel_index;

    // Use the next voxel if it has the upcoming index.
    // 点云遍历完毕，接着遍历voxel中的点
    // 或者当前voxel的index小于点云点需要的voxel
    // index，也就是还没有找到当前点云应该在的那个voxel，接着遍历voxel中的点
    if ((p_it == p_end) || (v_it != v_end && v_it->index <= p_it->voxel_index)) {
      voxel_index = v_it->index; // 当前voxel index记录
      voxel_data.old_voxel = &(*v_it);
      ++v_it; // 指向下一个voxel
    } else    // 点云没有遍历完毕，并且找到了当前点应该属于的voxel index
    {
      voxel_index = p_it->voxel_index;
    }

    // Gather all the points that belong to the current voxel
    //
    while (p_it != p_end && p_it->voxel_index == voxel_index) {
      ++p_it;
    }
    voxel_data.points_end = p_it; // 存储voxel_data结束点在索引点云中的位置points_end

    // Create the voxel
    if (createVoxel_(voxel_index, voxel_data, new_voxels, *new_active_centroids,
                     *new_inactive_centroids)) {
      created_voxel_indices.push_back(
          new_active_centroids->size() -
          1); // 由inactive变成active的voxel的在active_centroids_下的index
    }
  }

  // Done! Save the new voxels and return the indices of the triggered
  // voxels.
  voxels_ = std::move(new_voxels);
  active_centroids_ = std::move(new_active_centroids);
  inactive_centroids_ = std::move(new_inactive_centroids);
  return created_voxel_indices;
}

/// \brief: 根据点云坐标点计算每一个点的index
template <_DVG_TEMPLATE_DECL_>
template <typename PointXYZ_>
inline IndexT DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::getIndexOf(const PointXYZ_ &point) const {
  // 确认每个点都具有xyz三个坐标值
  static_assert(pcl::traits::has_xyz<PointXYZ_>::value,
                "PointXYZ_ must be a structure containing XYZ coordinates");
  // TODO: One could pack indexing transformation, offsetting and scaling in a single
  // transformation. min_corner and max_corner would need to be transformed as well in order
  // to allow checks. Since it would decrease readability significantly, this should be done only
  // if optimization is really necessary.

  // Transform the point back to the grid frame for hashing.
  // 将点云从sensor frame变换到map frame
  Eigen::Vector3f transformed_coords = indexing_transformation_.transform(point.getVector3fMap());

  // Ensure that the transformed point lies inside the grid.
  // 确认每个点都在珊格范围内
  CHECK(min_corner_(0) <= transformed_coords.x() && transformed_coords.x() < max_corner_(0));
  CHECK(min_corner_(1) <= transformed_coords.y() && transformed_coords.y() < max_corner_(1));
  CHECK(min_corner_(2) <= transformed_coords.z() && transformed_coords.z() < max_corner_(2));

  // Compute voxel index of the point.
  // 转换到珊格坐标系下
  Eigen::Vector3f grid_coords = (transformed_coords + indexing_offset_) * world_to_grid_;
  // 根据论文公式（1）计算珊格坐标系下标索引
  return static_cast<IndexT>(grid_coords[0]) + (static_cast<IndexT>(grid_coords[1]) << bits_x) +
         (static_cast<IndexT>(grid_coords[2]) << (bits_x + bits_y));
}

template <_DVG_TEMPLATE_DECL_>
void DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::transform(
    const kindr::minimal::QuatTransformationTemplate<float> &transformation) {
  // BENCHMARK_BLOCK("SM.TransformLocalMap.TransformDVG");

  // Update transforms
  pose_transformation_ = transformation * pose_transformation_;
  indexing_transformation_ = pose_transformation_.inverse();

  // Transform point clouds in-place
  for (auto centroids : {std::ref(active_centroids_), std::ref(inactive_centroids_)}) {
    for (auto &point : *centroids.get()) {
      point.getVector3fMap() = transformation.transform(point.getVector3fMap());
    }
  }
}

template <_DVG_TEMPLATE_DECL_> void DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::clear() {
  // Reset transformations.
  pose_transformation_.setIdentity();
  indexing_transformation_.setIdentity();

  // Clear points and voxels.
  active_centroids_->clear();
  inactive_centroids_->clear();
  voxels_.clear();
}

template <_DVG_TEMPLATE_DECL_> void DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::dumpVoxels() const {
  for (const Voxel_ &v : voxels_) {
    LOG(INFO) << "Voxel " << uint32_t(v.index) << ": " << v.num_points << " " << *(v.centroid);
  }
}

/// \brief: 给点云计算索引，并将点云按照voxel index升序进行排序后返回IndexedPoints_形式索引点云
template <_DVG_TEMPLATE_DECL_>
inline typename DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::IndexedPoints_
DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::indexAndSortPoints_(const InputCloud &points) const {
  IndexedPoints_ indexed_points; // vector<IndexedPoint_>，IndexedPoint_包含point和voxel_index
  indexed_points.reserve(points.size());

  // 将点云中每个点计算索引后插入到IndexedPoints_中，使用vector<IndexPoint_>存储
  for (const auto &point : points) {
    indexed_points.emplace_back(point, getIndexOf(point));
  }

  // 定义按照索引比较的lambda函数，并按照voxel index升序进行排序
  auto predicate = [](const IndexedPoint_ &a, const IndexedPoint_ &b) {
    return a.voxel_index < b.voxel_index;
  };
  std::sort(indexed_points.begin(), indexed_points.end(), predicate);

  return indexed_points;
}

template <_DVG_TEMPLATE_DECL_>
inline bool DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::createVoxel_(
    const IndexT index, const VoxelData_ &data, std::vector<Voxel_> &new_voxels,
    VoxelCloud &new_active_centroids, VoxelCloud &new_inactive_centroids) {
  VoxelPointT centroid;
  auto centroid_map = centroid.getVector3fMap(); //使用的Eigen::Map操作，相当于引用
  uint32_t old_points_count = 0u;
  uint32_t new_points_count = std::distance(data.points_begin, data.points_end);

  // Add contribution from the existing voxel.
  if (data.old_voxel != nullptr) {
    centroid = *(data.old_voxel->centroid);
    old_points_count = data.old_voxel->num_points;
    if (new_points_count != 0u) {
      centroid_map *= static_cast<float>(old_points_count);
    }
  }
  uint32_t total_points_count = old_points_count + new_points_count;

  // Add contribution from the new points.
  if (new_points_count != 0u) {
    for (auto it = data.points_begin; it != data.points_end; ++it) {
      centroid_map += it->point.getVector3fMap();
    }
    centroid_map /= static_cast<float>(total_points_count);
  }

  // Save centroid to the correct point cloud.
  VoxelPointT *centroid_pointer;
  bool is_new_voxel = false;
  if (total_points_count >= min_points_per_voxel_) // 当前是active
  {
    new_active_centroids.push_back(centroid);
    centroid_pointer = &new_active_centroids.back();
    is_new_voxel = (old_points_count < min_points_per_voxel_); // 是否是一个新增加的voxel
  } else {
    new_inactive_centroids.push_back(centroid);
    centroid_pointer = &new_inactive_centroids.back();
  }

  new_voxels.emplace_back(centroid_pointer, index, total_points_count);
  return is_new_voxel;
}

template <_DVG_TEMPLATE_DECL_>
inline std::vector<bool>
DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::removeCentroids_(VoxelCloud &target_cloud,
                                                        std::vector<VoxelPointT *> to_remove) {
  std::vector<bool> is_removed(target_cloud.size(), false);
  if (to_remove.empty())
    return is_removed;

  size_t next_removal_index = 0u;
  size_t centroid_index = 0u;

  // Push one more element so that we don't read past the end of the vector.
  to_remove.push_back((VoxelPointT *)0);

  // Remove the required centroids and keep track of their indices.
  auto new_end = std::remove_if(target_cloud.begin(), target_cloud.end(), [&](VoxelPointT &p) {
    const bool remove_p = (&p == to_remove[next_removal_index]);
    if (remove_p) {
      is_removed[centroid_index] = true;
      ++next_removal_index;
    }
    ++centroid_index;
    return remove_p;
  });
  target_cloud.erase(new_end, target_cloud.end());

  return is_removed;
}

} // namespace mapping
} // namespace long_term_relocalization

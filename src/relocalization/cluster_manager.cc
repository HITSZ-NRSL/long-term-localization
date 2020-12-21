// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/11/2.

#include "relocalization/cluster_manager.h"

namespace long_term_relocalization {

//============================================================
// ClustersManager Implementation
//============================================================
ClustersManager::ClustersManager()
    : centroids_cloud_(new PointCloud()), centroids2d_cloud_(new PointCloud2d()),
      global_cluster_id_(0) {
  clusters_.reserve(10000);
}

void ClustersManager::AddNewCluster(const ClustersManager::PointCloud::Ptr &cloud_in,
                                    int times_of_observed) {
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_in, centroid);
  this->NextCentroid() = common::Vector4fToPoint3d<PointT>(centroid);
  this->NextCentroid2d() = common::Vector4fToPoint2d(centroid);

  const int cluster_id = this->NextId();
  Cluster::Ptr cluster(new Cluster(cloud_in, cluster_id, this, times_of_observed));
  clusters_.push_back(cluster);

  CHECK_EQ(global_cluster_id_, centroids_cloud_->size());
  CHECK_EQ(global_cluster_id_, centroids2d_cloud_->size());
  CHECK_EQ(global_cluster_id_, cluster_id + 1);
  CHECK_EQ(global_cluster_id_, clusters_.size());
}

void ClustersManager::MergeCloudToOldCluster(const ClustersManager::PointCloud::Ptr &cloud_in,
                                             int old_cluster_id) {
  CHECK_LT(old_cluster_id, global_cluster_id_);
  clusters_[old_cluster_id]->MergeCloud(cloud_in);
  // LOG(INFO) << "Merge to old cluster.";
}

const Cluster::Ptr &ClustersManager::at(int id) const {
  CHECK_LT(id, global_cluster_id_);
  return clusters_.at(id);
}

Cluster::Ptr &ClustersManager::at(int id) {
  CHECK_LT(id, global_cluster_id_);
  return clusters_.at(id);
}

ClustersManager::PointCloud::Ptr ClustersManager::GetClustersCloud() {
  PointCloud::Ptr cloud_all(new PointCloud());

  if (local_clusters_indices_.empty()) {
    for (const auto &cluster : clusters_) {
      *cloud_all += cluster->cloud();
    }
  } else {
    std::unique_lock lock(local_clusters_indices_mutex_);
    for (const auto &index : local_clusters_indices_) {
      const Cluster::Ptr &cluster = this->at(index);
      *cloud_all += cluster->cloud();
    }
  }

  return cloud_all;
}

ClustersManager::PointT &ClustersManager::NextCentroid() {
  centroids_cloud_->push_back(PointT());
  return centroids_cloud_->back();
}

ClustersManager::Point2d &ClustersManager::NextCentroid2d() {
  centroids2d_cloud_->push_back(Point2d());
  return centroids2d_cloud_->back();
}

void ClustersManager::SaveClustersToFile(const std::string &filename) {
  std::ofstream outfile(filename, std::ios_base::binary);
  CHECK(outfile.is_open() && outfile.good());
  boost::archive::binary_oarchive binary_oarchive(outfile, boost::archive::no_header);

  LOG(INFO) << GREEN << "Begin to save clusters to " << filename << COLOR_END;
  const int num_static_clusters = GetNumStaticClusters();
  LOG(INFO) << "Number clusters: " << num_static_clusters;
  binary_oarchive << num_static_clusters;
  for (auto &cluster : clusters_) {
    if (cluster->is_static_pole()) {
      const int times_of_observed = cluster->times_of_observed();
      binary_oarchive << cluster->cloud() << times_of_observed;
    }
  }
  LOG(INFO) << GREEN << "Save clusters to file done ..." << COLOR_END;
}

void ClustersManager::LoadClusters(const std::string &filename, double remain_ratio) {

  std::ifstream infile(filename, std::ios_base::binary);
  CHECK(infile.is_open() && infile.good());
  boost::archive::binary_iarchive binary_iarchive(infile, boost::archive::no_header);
  LOG(INFO) << GREEN << "Begin to load clusters from " << filename << COLOR_END;
  int num_clusters = 0;
  binary_iarchive >> num_clusters;
  LOG(INFO) << "Number clusters: " << num_clusters;

  std::set<int> reserve_indices = RandomDownsamplingArray(num_clusters, remain_ratio);

  for (int i = 0; i < num_clusters; ++i) {
    PointCloud::Ptr cloud(new PointCloud());
    int times_of_observed = 0;
    binary_iarchive >> *cloud >> times_of_observed;

    if (reserve_indices.count(i)) {
      this->AddNewCluster(cloud, times_of_observed);
    }
  }
  LOG(INFO) << GREEN << "After downsampling, num clusters: " << this->num_clusters() << COLOR_END;
  LOG(INFO) << GREEN << "Load clusters from file done ..." << COLOR_END;
}

void ClustersManager::ComputeMiddleOutDescriptors(double search_radius) {
  pcl::KdTreeFLANN<Point2d>::Ptr kdtree(new pcl::KdTreeFLANN<Point2d>());
  kdtree->setInputCloud(this->centroids2d_cloud());

  for (const Cluster::Ptr &cluster : this->clusters()) {
    std::vector<int> k_indices;
    std::vector<float> k_sqr_dists;
    kdtree->radiusSearch(cluster->centroid2d(), search_radius, k_indices, k_sqr_dists);
    if (k_indices.size() <= 1) {
      continue;
    }

    // The first neighbor is itself.
    k_indices.erase(k_indices.begin());
    k_sqr_dists.erase(k_sqr_dists.begin());

    const int num_neighbors = k_indices.size();
    const Point2d &O = cluster->centroid2d();
    MiddleOutDescriptor *descriptor = cluster->mutable_middle_out_descriptor();
    descriptor->reset();
    descriptor->id = cluster->id();
    descriptor->num_neighbors = num_neighbors;
    descriptor->neighbor_ids = k_indices;
    descriptor->edges.resize(num_neighbors);
    descriptor->data.resize(num_neighbors);
    std::transform(k_sqr_dists.begin(), k_sqr_dists.end(), descriptor->edges.begin(),
                   [](double x) { return std::sqrt(x); });
    for (int i = 0; i < num_neighbors; ++i) {
      descriptor->data[i].reserve(num_neighbors - 1);
      const Point2d &A = this->at(k_indices[i])->centroid2d();
      for (int j = 0; j < num_neighbors; ++j) {
        if (j == i) {
          continue;
        }
        const Point2d &B = this->at(k_indices[j])->centroid2d();
        const double angle =
            ComputeAngleWithReferenceAxis(O, A, B, descriptor->edges[i], descriptor->edges[j]);
        descriptor->data[i].emplace_back(descriptor->edges[j], angle);
      }
    }
  }
}

void ClustersManager::ComputeMiddleOutDescriptors(double search_radius, const Point2d &cur_pose,
                                                  int local_clusters_size) {
  pcl::KdTreeFLANN<Point2d>::Ptr kdtree(new pcl::KdTreeFLANN<Point2d>());

  // Prepare static clusters 2d centroids.
  ClustersManager::PointCloud2d::Ptr static_centroids_cloud;
  std::vector<int> static_centroids_indices;
  std::tie(static_centroids_cloud, static_centroids_indices) =
      GetStaticCentroids2dCloudWithIndices();

  // Search static neighbors.
  std::vector<float> k_sqr_dists;
  std::vector<int> k_indices;
  kdtree->setInputCloud(static_centroids_cloud);
  kdtree->nearestKSearch(cur_pose, local_clusters_size, k_indices, k_sqr_dists);

  // Output static neighbors' cluster id.
  std::unique_lock lock(local_clusters_indices_mutex_);
  local_clusters_indices_.resize(k_indices.size());
  std::transform(
      k_indices.begin(), k_indices.end(), local_clusters_indices_.begin(),
      [&static_centroids_indices](int k_index) { return static_centroids_indices[k_index]; });

  LOG(INFO) << "local clusters size: " << local_clusters_indices_.size();

  CHECK(!local_clusters_indices_.empty());

  for (const auto &index : local_clusters_indices_) {
    const Cluster::Ptr &cluster = this->at(index);
    kdtree->radiusSearch(cluster->centroid2d(), search_radius, k_indices, k_sqr_dists);
    if (k_indices.size() <= 1) {
      continue;
    }

    std::vector<int> neighbor_indices(k_indices.size());
    std::transform(
        k_indices.begin(), k_indices.end(), neighbor_indices.begin(),
        [&static_centroids_indices](int k_index) { return static_centroids_indices[k_index]; });

    // The first neighbor is itself.
    neighbor_indices.erase(neighbor_indices.begin());
    k_sqr_dists.erase(k_sqr_dists.begin());

    const int num_neighbors = neighbor_indices.size();
    const Point2d &O = cluster->centroid2d();
    MiddleOutDescriptor *descriptor = cluster->mutable_middle_out_descriptor();
    descriptor->reset();
    descriptor->id = cluster->id();
    descriptor->num_neighbors = num_neighbors;
    descriptor->neighbor_ids = neighbor_indices;
    descriptor->edges.resize(num_neighbors);
    descriptor->data.resize(num_neighbors);
    std::transform(k_sqr_dists.begin(), k_sqr_dists.end(), descriptor->edges.begin(),
                   [](double x) { return std::sqrt(x); });
    for (int i = 0; i < num_neighbors; ++i) {
      descriptor->data[i].reserve(num_neighbors - 1);
      const Point2d &A = this->at(neighbor_indices[i])->centroid2d();
      for (int j = 0; j < num_neighbors; ++j) {
        if (j == i) {
          continue;
        }
        const Point2d &B = this->at(neighbor_indices[j])->centroid2d();
        const double angle =
            ComputeAngleWithReferenceAxis(O, A, B, descriptor->edges[i], descriptor->edges[j]);
        descriptor->data[i].emplace_back(descriptor->edges[j], angle);
      }
    }
  }
}

double ClustersManager::ComputeAngleWithReferenceAxis(const ClustersManager::Point2d &O,
                                                      const ClustersManager::Point2d &A,
                                                      const ClustersManager::Point2d &B, double OA,
                                                      double OB) {
  const double OA_x = A.x - O.x;
  const double OA_y = A.y - O.y;
  const double OB_x = B.x - O.x;
  const double OB_y = B.y - O.y;

  double angle = std::acos((OA_x * OB_x + OA_y * OB_y) / (OA * OB));
  const double cross_product = OA_x * OB_y - OA_y * OB_x;
  if (cross_product < 0) {
    angle *= -1;
  }

  return angle;
}

ClustersManager::PointCloud::Ptr ClustersManager::GetStaticClustersCloud() {
  PointCloud::Ptr cloud_all(new PointCloud());

  // if (local_clusters_indices_.empty()) {
  for (const auto &cluster : clusters_) {
    if (cluster->is_static_pole()) {
      *cloud_all += cluster->cloud();
    }
  }
  // } else {
  //   std::unique_lock lock(local_clusters_indices_mutex_);
  //   for (const auto &index : local_clusters_indices_) {
  //     const Cluster::Ptr &cluster = this->at(index);
  //     if (cluster->is_static_pole()) {
  //       *cloud_all += cluster->cloud();
  //     }
  //   }
  // }

  return cloud_all;
}

ClustersManager::PointCloud::Ptr ClustersManager::GetStaticCentroidsCloud() {
  PointCloud::Ptr centroids_cloud(new PointCloud());
  centroids_cloud->reserve(num_clusters());
  // if (local_clusters_indices_.empty()) {
  for (const auto &cluster : clusters_) {
    if (cluster->is_static_pole()) {
      centroids_cloud->push_back(cluster->centroid());
    }
  }
  // } else {
  //   std::unique_lock lock(local_clusters_indices_mutex_);
  //   for (const auto &index : local_clusters_indices_) {
  //     const Cluster::Ptr &cluster = this->at(index);
  //     if (cluster->is_static_pole()) {
  //       centroids_cloud->push_back(cluster->centroid());
  //     }
  //   }
  // }

  return centroids_cloud;
}
std::pair<ClustersManager::PointCloud2d::Ptr, std::vector<int>>
ClustersManager::GetStaticCentroids2dCloudWithIndices() {
  PointCloud2d::Ptr centroids2d_cloud(new PointCloud2d());
  centroids2d_cloud->reserve(num_clusters());
  std::vector<int> indices;
  indices.reserve(num_clusters());
  for (const auto &cluster : clusters_) {
    if (cluster->is_static_pole()) {
      centroids2d_cloud->push_back(cluster->centroid2d());
      indices.push_back(cluster->id());
    }
  }
  return {centroids2d_cloud, indices};
}

int ClustersManager::GetNumStaticClusters() {
  int num_static_clusters = 0;
  for (const auto &cluster : clusters_) {
    if (cluster->is_static_pole()) {
      ++num_static_clusters;
    }
  }

  return num_static_clusters;
}

} // namespace long_term_relocalization

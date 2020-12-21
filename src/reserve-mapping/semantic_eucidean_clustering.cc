#include "relocalization/semantic_eucidean_clustering.h"

#include <glog/logging.h>

#include <Eigen/Dense>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>

#include "utils/common/tic_toc.h"

namespace long_term_relocalization {


SemanticEucideanClustering::SemanticEucideanClustering(
    const params::SemanticEucideanClusteringParams &params)
    : params_(params) {}

void SemanticEucideanClustering::Process(const PointCloud &cloud_in) {
  ResetParams();
  ExtractSemanticRawClusters(cloud_in);
  MergeSemanticSeedsClusters();
}

// NOTE: buildings and other-objects are too large, if we use EucideanSegmentation, our system may
// not be RT any more.
void SemanticEucideanClustering::ExtractSemanticRawClusters(const PointCloud &cloud_in) {
  if (cloud_in.empty()) {
    return;
  }

  FUNC_TIME_BEGIN;
  // Filter point by label.
  for (const auto &point : cloud_in.points) {
    if (params_.object_eucidean_params.count(point.label) == 0) {
      continue;
    }
    seed_points_[point.label]->push_back(point);
  }

  // Do eucidean segmentation for each kind of semantic seed clusters.
  for (auto &item : seed_points_) {
    if (item.second->empty()) {
      continue;
    }
    SEGMENT_TIME_BEGIN(segmentation);
    const std::vector<pcl::PointIndices> clusters_indices =
        EucideanSegmentation(item.second, params_.object_eucidean_params[item.first]);
    SEGMENT_TIME_END(segmentation);

    SEGMENT_TIME_BEGIN(extract);
    PointCloud::Ptr points_of_one_label(new PointCloud());
    for (const auto &indices : clusters_indices) {
      PointCloud::Ptr cloud(new PointCloud());
      pcl::copyPointCloud(*item.second, indices, *cloud);

      // Modify intensity for visualization.
      const int intensity = std::rand() % 255;
      for (auto &point : cloud->points) {
        point.intensity = intensity;
      }
      *points_of_one_label += *cloud;

      const Cluster::Ptr cluster(new Cluster(cloud));
      raw_clusters_[item.first].push_back(cluster);
    }
    item.second = points_of_one_label;
    SEGMENT_TIME_END(extract);
  }
  FUNC_TIME_END;
}

void SemanticEucideanClustering::MergeSemanticSeedsClusters() {
  // double angleToVertical, double maxDistanceStitches, list<Segment> stitchedClusters;
  // // Sort all clusters by height.
  // boost::random::uniform_int_distribution<> dist(0, 255);
  // rawClusters->sort(compareClusterHeight);

  // /* To check the input clusters
  // int j = 0;
  // for (list<Cluster>::iterator it = rawClusters.begin(); it != rawClusters.end(); ++it){
  //     Cluster cluster = *it;
  //     cerr << "Radius: " << cluster.getRadius() << endl;
  //     cerr << "Z min: " << cluster.getZMin() << endl;
  //     cerr << "Z max: " << cluster.getZMax() << endl;
  //     pointCloudVisualizer(cluster.getClusterCloud(), "cluster cloud " +
  // boost::lexical_cast<string>(j)); pointCloudVisualizer(cluster, "cluster shape " +
  // boost::lexical_cast<string>(j)); j++; break;
  // }
  // */

  // int numClusters = 0;
  // for (list<Cluster>::iterator it = rawClusters->begin(); it != rawClusters->end(); ++it) {
  //   if (!it->isProcessed()) {
  //     it->markProcessed();
  //     Segment stitchCluster;
  //     stitchCluster.addCluster(*it);
  //     // pointCloudVisualizer(it->getClusterCloud(), "cluster cloud " +
  //     // boost::lexical_cast<string>(j));
  //     // pointCloudVisualizer(*it, "cluster shape " + boost::lexical_cast<string>(j));
  //     for (list<Cluster>::iterator it2 = rawClusters->begin(); it2 != rawClusters->end(); ++it2) {
  //       if (!it2->isProcessed()) {
  //         // pointCloudVisualizer(it2->getClusterCloud(), "cluster cloud " +
  //         // boost::lexical_cast<string>(j)); pointCloudVisualizer(*it2, "cluster shape " +
  //         // boost::lexical_cast<string>(j));
  //         double heightDiff = it2->getZMin() - stitchCluster.getZMax();
  //         // double heightDiff = it2->getCentroid()[2] - stitchCluster.getCentroid()[2];
  //         // double heightDiff = it2->getCentroid()[2] - stitchCluster.getZMax();
  //         // 1. centroids' height diff
  //         // 2. centroids's vector norm and angle
  //         if (heightDiff < maxDistanceStitches && heightDiff > 0) {
  //           Eigen::Vector4f diffVec = it2->getCentroid() - stitchCluster.getCentroid();
  //           //** Hardcoded value below for Euclidean distance threshold of clusters to be stitched
  //           if (diffVec.norm() < 5) {
  //             // double angle =
  //             // abs(pcl::normAngle(diffVec.dot(Eigen::Vector4f::UnitZ())/diffVec.norm()));
  //             double angle = pcl::getAngle3D(diffVec, Eigen::Vector4f::UnitZ());
  //             if (angle < angleToVertical) {
  //               /* Debugs
  //               cerr << "heightDiff: " << heightDiff << endl;
  //               cerr << "Euclidean distance: " << diffVec.norm() << endl;
  //               cerr << "Angle: " << angle << endl;
  //               */
  //               it2->markProcessed();
  //               stitchCluster.addCluster(*it2);
  //             }
  //           }
  //         }
  //       }
  //     }
  //     stitchedClusters.push_back(stitchCluster);
  //   }
  // }
  // cerr << "Number of clusters after stitching: " << stitchedClusters.size() << endl;
}

std::vector<pcl::PointIndices> SemanticEucideanClustering::EucideanSegmentation(
    const PointCloud::Ptr &cloud,
    params::SemanticEucideanClusteringParams::ObjectEucideanParams &obj_eucidean_params) {
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusters_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(obj_eucidean_params.euclidean_radius);
  ec.setMinClusterSize(obj_eucidean_params.min_cluster_size);
  ec.setMaxClusterSize(cloud->size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters_indices);

  return clusters_indices;
}

std::vector<pcl::PointIndices>
SemanticEucideanClustering::SemanticEucideanSegmentation(const PointCloud::Ptr &cloud,
                                                         const std::vector<int> &seed_indices) {
  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl_utils::PointIRL>::Ptr tree(new pcl::search::KdTree<pcl_utils::PointIRL>);
  tree->setInputCloud(cloud);

  // Check if the tree is sorted -- if it is we don't need to check the first element
  int nn_start_idx = tree->getSortedResults() ? 1 : 0;

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed(cloud->points.size(), false);
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  // Process all seed points in the indices vector
  for (int i = 0; i < seed_indices.size(); ++i) {
    if (processed[seed_indices[i]])
      continue;

    const auto &seed_point = cloud->points[seed_indices[i]];

    std::vector<int> q;
    int q_idx = 0;
    q.push_back(seed_indices[i]);
    processed[seed_indices[i]] = true;

    while (q_idx < q.size()) {
      const int ret = tree->radiusSearch(cloud->points[q[q_idx]], params_.euclidean_radius,
                                         nn_indices, nn_distances);
      CHECK_NE(ret, -1) << "Received error code -1 from radiusSearch";

      if (ret == 0) {
        q_idx++;
        continue;
      }

      // can't assume sorted (default isn't!)
      for (size_t j = nn_start_idx; j < nn_indices.size(); ++j) {
        if (nn_indices[j] == -1 || processed[nn_indices[j]])
          continue;

        // Perform a simple Euclidean clustering
        q.push_back(nn_indices[j]);
        processed[nn_indices[j]] = true;
      }

      q_idx++;
    }

    // Add to the clusters
    clusters.emplace_back();
    pcl::PointIndices &r = clusters.back();
    r.indices.resize(q.size());
    for (size_t j = 0; j < q.size(); ++j) {
      // This is the only place where indices come into play
      r.indices[j] = q[j];
    }
    r.header = cloud->header;
  }

  return clusters;
}

bool SemanticEucideanClustering::IsPoleLikeCluster(const PointCloud &cluster) {
  if (cluster.size() <= params_.min_cluster_size) {
    return false;
  }

  Eigen::Vector4f min_point, max_point;
  pcl::getMinMax3D(cluster, min_point, max_point);

  const Eigen::Vector4f diff_point = max_point - min_point;
  const double xy_bound =
      std::sqrt(diff_point.x() * diff_point.x() + diff_point.y() * diff_point.y());
  const double height = std::abs(max_point.z() - min_point.z());

  return xy_bound <= params_.max_pole_xy_bound && height > params_.min_pole_height;
}

void SemanticEucideanClustering::ResetParams() {
  seed_points_.clear();
  for (const auto &item : params_.object_eucidean_params) {
    seed_points_[item.first] = boost::make_shared<PointCloud>();
  }

  raw_clusters_.clear();
}


} // namespace long_term_relocalization

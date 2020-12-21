// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/11/2.

#pragma once

#include "relocalization/cluster_manager.h"
#include "utils/params/params_types.h"

namespace long_term_relocalization {

using Id = int64_t;
using IdPair = std::pair<Id, Id>;
using PointPair = std::pair<Cluster::PointT, Cluster::PointT>;
using PointPairs = std::vector<PointPair>;

/// \brief Struct providing an hashing function for pairs of segment IDs.
struct IdPairHash {
  /// \brief Hashing function for pairs of segment IDs.
  /// \param pair Pair of IDs to be hashed.
  /// \returns The hash of the ID pair.
  size_t operator()(const IdPair &pair) const {
    static_assert(std::is_same<IdPair, std::pair<int64_t, int64_t>>::value,
                  "The hashing function is valid only if IdPair is defined as "
                  "std::pair<int64_t, int64_t>");
    // We expect IDs to be always positive, which enables this trick for combining the two IDs. If
    // that would not be the case the hashing function could be less efficient, but still
    // functional.
    return std::hash<uint64_t>{}(static_cast<uint64_t>(pair.first)
                                 << 1 + static_cast<uint64_t>(pair.second));
  }
};

class PairwiseMatch {
public:
  PairwiseMatch() = default;
  PairwiseMatch(const Cluster::Ptr &source, const Cluster::Ptr &target, float confidence_in = 1.0)
      : ids_(source->id(), target->id()), confidence_(confidence_in),
        centroids_(PointPair(source->centroid(), target->centroid())) {}

  PairwiseMatch(Id id1 /*source*/, Id id2 /*target*/, const Cluster::PointT &centroid1,
                const Cluster::PointT &centroid2, float confidence_in = 1.0)
      : ids_(id1, id2), confidence_(confidence_in), centroids_(PointPair(centroid1, centroid2)) {}

  PointPair centroids() const { return centroids_; }
  IdPair ids_;
  float confidence_;
  Eigen::MatrixXd features1_;
  Eigen::MatrixXd features2_;
  PointPair centroids_;
};

using PairwiseMatches = std::vector<PairwiseMatch, Eigen::aligned_allocator<PairwiseMatch>>;

struct EdgePairWiseMatch {
  PairwiseMatch src_edge;
  PairwiseMatch tar_edge;
};

using EdgePairWiseMatches =
    std::vector<EdgePairWiseMatch, Eigen::aligned_allocator<EdgePairWiseMatch>>;

class ClusterMatcher {
public:
  ClusterMatcher(const params::ClusterMatcherParams &params) : params_(params) {}

  virtual ~ClusterMatcher() = default;

  bool AreTwoClustersMatched(const Cluster::Ptr &source_cluster,
                             const Cluster::Ptr &target_cluster);

private:
  double ComputeDistanceBetweenDistAngles(const DistAngle &lhs, const DistAngle &rhs);

  int ComputeDistanceBetweenNeighborEdges(const std::vector<DistAngle> &source,
                                          const std::vector<DistAngle> &target, double *distance);

  params::ClusterMatcherParams params_;
};

} // namespace long_term_relocalization

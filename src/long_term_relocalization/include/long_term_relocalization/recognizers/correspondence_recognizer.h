// Copyright (c) eth-asl. All rights reserved.
// https://github.com/ethz-asl/segmap/tree/master/segmatch/src/recognizers

#pragma once

#include <vector>

#include "long_term_relocalization/relocalization/cluster_matcher.h"

namespace long_term_relocalization {

/// \brief Base class for recognizing a model in a scene.
class CorrespondenceRecognizer {
public:
  /// \brief Finalizes an instance of the CorrespondenceRecognizer class.
  virtual ~CorrespondenceRecognizer() = default;

  /// \brief Sets the current matches and tries to recognize the model.
  /// \param predicted_matches Vector of possible correspondences between model and scene.
  virtual void recognize(const PairwiseMatches &predicted_matches) = 0;

  /// \brief Gets the candidate transformations between model and scene.
  /// \returns Vector containing the candidate transformations. Transformations are sorted in
  /// decreasing recognition quality order. If empty, the model was not recognized.
  virtual const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &
  getCandidateTransformations() const = 0;

  /// \brief Gets the candidate clusters of matches between model and scene. Every cluster
  /// represents a possible recognition.
  /// \returns Vector containing the candidate clusters. Clusters are sorted in
  /// decreasing recognition quality order. If empty, the model was not recognized.
  virtual const std::vector<PairwiseMatches> &getCandidateClusters() const = 0;
}; // class CorrespondenceRecognizer

} // namespace long_term_relocalization

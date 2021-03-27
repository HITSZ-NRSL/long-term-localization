// Copyright (c) eth-asl. All rights reserved.
// https://github.com/ethz-asl/segmap/tree/master/segmatch/src/recognizers

#pragma once

#include "long_term_relocalization/recognizers/correspondence_recognizer.h"
#include "long_term_relocalization/utils/params_types.h"

namespace long_term_relocalization {

/// \brief Factory class for correspondence recognizers.
class CorrespondenceRecognizerFactory {
public:
  /// \brief Initializes a new instance of the CorrespondenceRecognizerFactory class.
  /// \param params The current parameters of SegMatch.
  CorrespondenceRecognizerFactory(const GeometricConsistencyParams &params);

  /// \brief Creates a correspondence recognizer.
  /// \returns Pointer to a new CorrespondencdRecognizer instance.
  std::unique_ptr<CorrespondenceRecognizer> create() const;

private:
  GeometricConsistencyParams params_;
}; // class CorrespondenceRecognizerFactory

} // namespace long_term_relocalization

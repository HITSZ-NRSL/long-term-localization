#pragma once

#include "relocalization/recognizers/correspondence_recognizer.h"
#include "utils/params/params_types.h"

namespace long_term_relocalization {


/// \brief Factory class for correspondence recognizers.
class CorrespondenceRecognizerFactory {
public:
  /// \brief Initializes a new instance of the CorrespondenceRecognizerFactory class.
  /// \param params The current parameters of SegMatch.
  CorrespondenceRecognizerFactory(const params::GeometricConsistencyParams &params);

  /// \brief Creates a correspondence recognizer.
  /// \returns Pointer to a new CorrespondencdRecognizer instance.
  std::unique_ptr<CorrespondenceRecognizer> create() const;

private:
  params::GeometricConsistencyParams params_;
}; // class CorrespondenceRecognizerFactory


} // namespace long_term_relocalization

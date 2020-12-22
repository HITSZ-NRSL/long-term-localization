#include "long_term_relocalization/recognizers/correspondence_recognizer_factory.h"

#include "long_term_relocalization/recognizers/geometric_consistency_recognizer.h"
#include "long_term_relocalization/recognizers/incremental_geometric_consistency_recognizer.h"
#include "long_term_relocalization/recognizers/partitioned_geometric_consistency_recognizer.h"

namespace long_term_relocalization {


CorrespondenceRecognizerFactory::CorrespondenceRecognizerFactory(
    const GeometricConsistencyParams &params)
    : params_(params) {}

std::unique_ptr<CorrespondenceRecognizer> CorrespondenceRecognizerFactory::create() const {
  if (params_.recognizer_type == "Simple") {
    return std::unique_ptr<CorrespondenceRecognizer>(new GeometricConsistencyRecognizer(params_));
  } else if (params_.recognizer_type == "Partitioned") {
    return std::unique_ptr<CorrespondenceRecognizer>(
        new PartitionedGeometricConsistencyRecognizer(params_, params_.local_map_radius));
  } else if (params_.recognizer_type == "Incremental") {
    return std::unique_ptr<CorrespondenceRecognizer>(
        new IncrementalGeometricConsistencyRecognizer(params_, params_.local_map_radius));
  } else {
    LOG(FATAL) << "Invalid recognizer type specified: " << params_.recognizer_type;
    throw std::invalid_argument("Invalid recognizer type specified: " + params_.recognizer_type);
  }
}


} // namespace long_term_relocalization

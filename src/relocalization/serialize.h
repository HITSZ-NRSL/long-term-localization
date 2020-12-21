// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/25.

#pragma once

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "utils/common/pcl_types.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive &ar, long_term_relocalization::pcl_utils::PointIRL &point, const unsigned int version) {
  ar &point.x;
  ar &point.y;
  ar &point.z;
  ar &point.intensity;
  ar &point.ring;
  ar &point.label;
}

template <class Archive>
void serialize(Archive &ar, long_term_relocalization::pcl_utils::PointIRLCloud &cloud,
               const unsigned int file_version) {
  ar &cloud.points;
}

} // namespace serialization
} // namespace boost

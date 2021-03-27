// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-1.

#pragma once

#include <glog/logging.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include "common/pcl_utils/pcl_types.h"
#include "long_term_relocalization/utils/params_types.h"

namespace long_term_relocalization {
namespace mapping {

template <typename PointT> class Preprocessor {
  using PointCloud = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;

public:
  explicit Preprocessor(const PreprocessorParams &params) {
    sor_ = boost::make_shared<pcl::StatisticalOutlierRemoval<PointT>>();
    sor_->setMeanK(params.sor_noise_meank);
    sor_->setStddevMulThresh(params.sor_noise_stddev_multhresh);
  }

  virtual ~Preprocessor() = default;

  void Process(const PointCloud &cloud_in, PointCloudPtr cloud_out) {
    CHECK(cloud_out != nullptr);
    sor_->setInputCloud(cloud_in.makeShared());
    sor_->filter(*cloud_out);
  }

private:
  typename pcl::StatisticalOutlierRemoval<PointT>::Ptr sor_;
};

} // namespace mapping
} // namespace long_term_relocalization

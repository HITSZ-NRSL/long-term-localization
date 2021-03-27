// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/10/19.

#include "long_term_relocalization/relocalization/relocalization.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "long_term_relocalization/utils/params_loader.h"

using namespace long_term_relocalization;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "semantic_relocalization_main");

  ros::NodeHandle nh("~");
  ParamsLoader params_loader(GetConfigFilePath(), GetRS80AngleFilePath());

  Relocalization reloclaization(params_loader.get_relocalization_params(), nh);

  ros::spin();
  return 0;
}
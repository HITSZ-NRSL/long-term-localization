// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/21.

#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

#include <glog/logging.h>

namespace common {

class TicToc {
public:
  TicToc() { tic(); }

  void tic() { start_ = std::chrono::system_clock::now(); }

  double toc() {
    end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_ - start_;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
};

} // namespace common

#define FUNC_TIME_BEGIN common::TicToc __func_tictoc;
#define FUNC_TIME_END LOG(INFO) << __FUNCTION__ << " cost time: " << __func_tictoc.toc() << " ms.";
#define FUNC_TIME_END_WHETHER_OVERTIME(time_limit_ms)                                              \
  LOG_IF(WARNING, __func_tictoc.toc() > time_limit_ms)                                             \
      << __FUNCTION__ << " cost time over: " << __func_tictoc.toc() << " ms.";

#define SEGMENT_TIME_BEGIN(segname) common::TicToc segname;
#define SEGMENT_TIME_END(segname)                                                                  \
  LOG(INFO) << (#segname) << " cost time: " << segname.toc() << " ms.";
#define SEGMENT_TIME_END_WHETHER_OVERTIME(segname, time_limit_ms)                                  \
  LOG_IF(WARNING, segname.toc() > time_limit_ms)                                                   \
      << (#segname) << " cost time over: " << segname.toc() << " ms.";

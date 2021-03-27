// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/26.

#pragma once

#include <algorithm>
#include <chrono>
#include <random>
#include <set>
#include <vector>

#include <glog/logging.h>

/*Output style*/
#define RESET "\033[0m"
#define GREEN "\033[32m"              ///< Green
#define YELLOW "\033[33m"             ///< Yellow
#define BLUE "\033[34m"               ///< Blue
#define MAGENTA "\033[35m"            ///< Magenta
#define CYAN "\033[36m"               ///< Cyan
#define BOLDBLACK "\033[1m\033[30m"   ///< Bold Black
#define BOLDRED "\033[1m\033[31m"     ///< Bold Red
#define BOLDGREEN "\033[1m\033[32m"   ///< Bold Green
#define BOLDYELLOW "\033[1m\033[33m"  ///< Bold Yellow
#define BOLDBLUE "\033[1m\033[34m"    ///< Bold Blue
#define BOLDMAGENTA "\033[1m\033[35m" ///< Bold Magenta
#define BOLDCYAN "\033[1m\033[36m"    ///< Bold Cyan
#define BOLDWHITE "\033[1m\033[37m"   ///< Bold White

#define COLOR_END "\033[0m"

namespace common {

template <typename Iterator>
void TraverseLogOut(Iterator begin, Iterator end, bool with_index = false) {
  if (!with_index) {
    while (begin != end) {
      LOG(INFO) << *begin << std::endl;
      ++begin;
    }
  } else {
    int index = 0;
    while (begin != end) {
      LOG(INFO) << index << ": " << *begin << std::endl;
      ++begin;
      ++index;
    }
  }
}

template <typename Container>
void TraverseLogOut(const Container &container, bool with_index = false) {
  TraverseLogOut(container.begin(), container.end(), with_index);
}

} // namespace common

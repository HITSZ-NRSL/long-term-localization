// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#include "common/fixed_array2d.h"

#include <gtest/gtest.h>

namespace common {

TEST(FixedArray2D, Constructor) {
  FixedArray2D<int> mat(100, 200, -1);
  EXPECT_EQ(100, mat.rows());
  EXPECT_EQ(200, mat.cols());

  for (int i = 0; i < 100; ++i) {
    for (int j = 0; j < 200; ++j) {
      ASSERT_EQ(mat(i, j), -1);
    }
  }

  std::fill(mat.begin(), mat.end(), 10);
  for (int i = 0; i < 100; ++i) {
    for (int j = 0; j < 200; ++j) {
      ASSERT_EQ(mat(i, j), 10);
    }
  }
}

TEST(FixedArray2D, CopyConstructor) {
  FixedArray2D<int> mat;
  EXPECT_TRUE(mat.empty());

  mat = FixedArray2D<int>(100, 400, -1);
  EXPECT_EQ(100, mat.rows());
  EXPECT_EQ(400, mat.cols());

  for (int i = 0; i < 100; ++i) {
    for (int j = 0; j < 400; ++j) {
      ASSERT_EQ(mat(i, j), -1);
    }
  }
}

} // namespace common

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  RUN_ALL_TESTS();

  return 0;
}
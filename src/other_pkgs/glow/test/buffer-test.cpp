#include <gtest/gtest.h>

#include <glow/GlBuffer.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include "test_utils.h"

using namespace glow;

namespace {

TEST(BufferTest, initTest) {
  GlBuffer<float> buffer(BufferTarget::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW);
  // Note: a buffer is not a buffer until the first time bound,
  // even though it was created with glGenBuffers.
  buffer.bind();
  buffer.release();
  ASSERT_TRUE(glIsBuffer(buffer.id()));
}

TEST(BufferTest, assignTest) {
  uint32_t num_values = 17;
  std::vector<float> values(num_values);
  Random rand(1234);
  for (uint32_t i = 0; i < values.size(); ++i) {
    values[i] = rand.getFloat();
  }

  GlBuffer<float> buffer(BufferTarget::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW);

  ASSERT_EQ(BufferTarget::ARRAY_BUFFER, buffer.target());
  ASSERT_EQ(BufferUsage::DYNAMIC_DRAW, buffer.usage());
  ASSERT_EQ(static_cast<size_t>(0), buffer.size());      // empty buffer.
  ASSERT_EQ(static_cast<size_t>(0), buffer.capacity());  // really empty buffer.

  buffer.assign(values);
  ASSERT_EQ(values.size(), buffer.size());  // now not empty buffer.

  std::vector<float> buf;
  buffer.get(buf);
  ASSERT_EQ(values.size(), buf.size());

  for (uint32_t i = 0; i < values.size(); ++i) {
    ASSERT_FLOAT_EQ(values[i], buf[i]);
  }

  GlBuffer<float> assign_buffer(BufferTarget::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW);

  assign_buffer.assign(buffer);

  buf.clear();
  assign_buffer.get(buf);
  ASSERT_EQ(values.size(), buf.size());

  for (uint32_t i = 0; i < values.size(); ++i) {
    ASSERT_FLOAT_EQ(values[i], buf[i]);
  }

  ASSERT_NO_THROW(CheckGlError());
}

TEST(BufferTest, reserveTest) {
  GlBuffer<float> buffer(BufferTarget::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW);

  buffer.reserve(1000);
  ASSERT_EQ(static_cast<size_t>(1000), buffer.capacity());
  ASSERT_EQ(static_cast<size_t>(0), buffer.size());

  ASSERT_NO_THROW(CheckGlError());
}

TEST(BufferTest, resizeTest) {
  GlBuffer<float> buffer(BufferTarget::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW);

  buffer.reserve(1000);
  ASSERT_EQ(static_cast<size_t>(1000), buffer.capacity());
  ASSERT_EQ(static_cast<size_t>(0), buffer.size());

  buffer.resize(123);
  ASSERT_EQ(static_cast<size_t>(123), buffer.size());
  ASSERT_EQ(static_cast<size_t>(1000), buffer.capacity());

  GlBuffer<float> buffer2(BufferTarget::ARRAY_BUFFER, BufferUsage::DYNAMIC_DRAW);
  buffer2.resize(145);
  ASSERT_EQ(static_cast<size_t>(145), buffer2.size());
  ASSERT_EQ(static_cast<size_t>(145), buffer2.capacity());

  ASSERT_NO_THROW(CheckGlError());
}

TEST(BufferTest, replaceTest) {
  GlBuffer<int32_t> buffer(BufferTarget::ARRAY_BUFFER, BufferUsage::STATIC_DRAW);

  uint32_t num_values = 100;
  std::vector<int32_t> values(num_values);

  Random rand(1234);
  for (uint32_t i = 0; i < values.size(); ++i) {
    values[i] = rand.getInt(100, 147);
  }

  buffer.assign(values);
  ASSERT_EQ(values.size(), buffer.size());

  std::vector<int32_t> buf_values;
  buffer.get(buf_values);
  ASSERT_EQ(values.size(), buf_values.size());

  // just to be sure that everything is as expected.
  for (uint32_t i = 0; i < buf_values.size(); ++i) {
    ASSERT_EQ(values[i], buf_values[i]);
  }

  ASSERT_NO_THROW(CheckGlError());

  std::vector<int32_t> replace_values(13, 1);

  uint32_t offset = 10;
  buffer.replace(offset, replace_values);
  ASSERT_EQ(values.size(), buffer.size());

  buffer.get(buf_values);
  for (uint32_t i = 0; i < buf_values.size(); ++i) {
    if (i < offset || i >= offset + replace_values.size()) {
      ASSERT_EQ(values[i], buf_values[i]);
    } else {
      ASSERT_EQ(replace_values[i - offset], buf_values[i]);
      values[i] = buf_values[i];  // ensure that values contains last buffer content.
    }
  }

  ASSERT_NO_THROW(CheckGlError());

  // CHECK BOUNDARY CASE.
  offset = 95;
  replace_values.assign(124, 2);
  buffer.replace(offset, replace_values);
  ASSERT_EQ(values.size(), buffer.size());

  ASSERT_NO_THROW(CheckGlError());

  buffer.get(buf_values);
  for (uint32_t i = 0; i < buf_values.size(); ++i) {
    if (i < offset) {
      ASSERT_EQ(values[i], buf_values[i]);
    } else {
      ASSERT_EQ(replace_values[i - offset], buf_values[i]);
      values[i] = buf_values[i];  // ensure that values contains last buffer content.
    }
  }

  ASSERT_NO_THROW(CheckGlError());
}

TEST(BufferTest, eigenTest) {
  GlBuffer<Eigen::Matrix4f> mats(BufferTarget::ARRAY_BUFFER, BufferUsage::STATIC_READ);

  std::vector<Eigen::Matrix4f> orig_mats;
  Eigen::Matrix4f t1;
  t1 << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;

  orig_mats.push_back(t1);
  Eigen::Matrix4f t2 = -1.0f * t1;
  orig_mats.push_back(t2);

//  std::cout << "sizeof(Eigen::Matrix4f) = " << sizeof(Eigen::Matrix4f) << ", "
//            << sizeof(Eigen::Matrix4f) / sizeof(float) << std::endl;

  //  for (uint32_t i = 0; i < 16; ++i) {
  //    std::cout << ((float*)&(orig_mats[0]) + i) << ",";
  //  }
//  std::cout << std::endl;

  mats.assign(orig_mats);
  std::vector<Eigen::Matrix4f> buffered_mats;
  mats.get(buffered_mats);

  ASSERT_EQ(orig_mats.size(), buffered_mats.size());

  for (uint32_t k = 0; k < buffered_mats.size(); ++k) {
//    std::cout << buffered_mats[k] << std::endl;
    const Eigen::Matrix4f& A = buffered_mats[k];
    const Eigen::Matrix4f& B = orig_mats[k];
    for (uint32_t i = 0; i < 4; ++i) {
      for (uint32_t j = 0; j < 4; ++j) {
        ASSERT_LT(std::abs(A(i, j) - B(i, j)), 0.001f);
      }
    }
  }
}
}

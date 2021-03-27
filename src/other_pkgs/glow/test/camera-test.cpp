#include <gtest/gtest.h>

#include <glow/RoSeCamera.h>
#include <glow/string_utils.h>

using namespace glow;

TEST(RoSeCameraTest, testSetMatrix) {
  float EPS = 0.0001;

  RoSeCamera cam;
  std::vector<float> gold_params(5);
  std::vector<float> result_params(5);

  cam.setPosition(1, 2, 3);
  Eigen::Matrix4f m_gold = cam.matrix();
  cam.getCameraParameters(gold_params[0], gold_params[1], gold_params[2], gold_params[3], gold_params[4]);

  cam.setMatrix(cam.matrix());
  Eigen::Matrix4f result = cam.matrix();
  cam.getCameraParameters(result_params[0], result_params[1], result_params[2], result_params[3], result_params[4]);

  ASSERT_FLOAT_EQ(gold_params[0], result_params[0]);  // x
  ASSERT_FLOAT_EQ(gold_params[1], result_params[1]);  // y
  ASSERT_FLOAT_EQ(gold_params[2], result_params[2]);  // z
  ASSERT_FLOAT_EQ(gold_params[3], result_params[3]);  // yaw
  ASSERT_FLOAT_EQ(gold_params[4], result_params[4]);  // pitch

  for (uint32_t i = 0; i < 4; ++i) {
    for (uint32_t j = 0; j < 4; ++j) {
      ASSERT_NEAR(m_gold(i, j), result(i, j), 0.00001) << "(" << i << ", " << j << ") expected: " << m_gold(i, j)
                                                       << ", but got: " << result(i, j);
    }
  }

  // more complicated example.

  cam.lookAt(5., 2., 1., 0., 0., 0.);

  m_gold = cam.matrix();
  cam.getCameraParameters(gold_params[0], gold_params[1], gold_params[2], gold_params[3], gold_params[4]);

  cam.setMatrix(cam.matrix());
  result = cam.matrix();

  cam.getCameraParameters(result_params[0], result_params[1], result_params[2], result_params[3], result_params[4]);

  //  std::cout << "gold = " << rv::stringify(gold_params) << std::endl;
  //  std::cout << "result = " << rv::stringify(result_params) << std::endl;

  ASSERT_FLOAT_EQ(gold_params[0], result_params[0]);  // x
  ASSERT_FLOAT_EQ(gold_params[1], result_params[1]);  // y
  ASSERT_FLOAT_EQ(gold_params[2], result_params[2]);  // z
  ASSERT_FLOAT_EQ(gold_params[3], result_params[3]);  // yaw
  ASSERT_FLOAT_EQ(gold_params[4], result_params[4]);  // pitch

  for (uint32_t i = 0; i < 4; ++i) {
    for (uint32_t j = 0; j < 4; ++j) {
      ASSERT_NEAR(m_gold(i, j), result(i, j), EPS) << "(" << i << ", " << j << ") expected: " << m_gold(i, j)
                                                   << ", but got: " << result(i, j);
    }
  }

  // even more complicated example.

  cam.lookAt(15., -22.3, 11.3, 2., -1., -1.);

  m_gold = cam.matrix();
  cam.getCameraParameters(gold_params[0], gold_params[1], gold_params[2], gold_params[3], gold_params[4]);

  cam.setMatrix(cam.matrix());
  result = cam.matrix();

  cam.getCameraParameters(result_params[0], result_params[1], result_params[2], result_params[3], result_params[4]);

  //  std::cout << "gold = " << rv::stringify(gold_params) << std::endl;
  //  std::cout << "result = " << rv::stringify(result_params) << std::endl;

  ASSERT_NEAR(gold_params[0], result_params[0], EPS);  // x
  ASSERT_NEAR(gold_params[1], result_params[1], EPS);  // y
  ASSERT_NEAR(gold_params[2], result_params[2], EPS);  // z
  ASSERT_NEAR(gold_params[3], result_params[3], EPS);  // yaw
  ASSERT_NEAR(gold_params[4], result_params[4], EPS);  // pitch

  for (uint32_t i = 0; i < 4; ++i) {
    for (uint32_t j = 0; j < 4; ++j) {
      ASSERT_NEAR(m_gold(i, j), result(i, j), EPS) << "(" << i << ", " << j << ") expected: " << m_gold(i, j)
                                                   << ", but got: " << result(i, j);
    }
  }
}

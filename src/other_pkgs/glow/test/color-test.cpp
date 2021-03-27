#include <gtest/gtest.h>
#include <glow/GlColor.h>
#include <glow/glutil.h>

using namespace glow;

namespace {

TEST(ColorTest, testConversionFromHSV) {
  // values generated with GIMP.
  GlColor col = GlColor::FromHSV(radians(193), 45.0f / 100.f, 54.0f / 100.0f);
  ASSERT_NEAR(76.0f / 255.0f, col.R, 0.01f);
  ASSERT_NEAR(124.0f / 255.0f, col.G, 0.01f);
  ASSERT_NEAR(138.0f / 255.0f, col.B, 0.01f);

  GlColor col0 = GlColor::FromHSV(radians(193), 45.0f / 100.f, 0);
  ASSERT_NEAR(0.0f, col0.R, 0.01);
  ASSERT_NEAR(0.0f, col0.G, 0.01f);
  ASSERT_NEAR(0.0f, col0.B, 0.01f);

  GlColor col1 = GlColor::FromHSV(radians(13), 82.0f / 100.f, 88.0f / 100.0f);
  ASSERT_NEAR(223.0f / 255.0f, col1.R, 0.01f);
  ASSERT_NEAR(79.0f / 255.0f, col1.G, 0.01f);
  ASSERT_NEAR(40.0f / 255.0f, col1.B, 0.01f);
}

TEST(ColorTest, testBrightness) {
  GlColor col = GlColor::FromHSV(radians(193), 45.0f / 100.f, 54.0f / 100.0f);
  col.lighter();
  ASSERT_NEAR(1.42 * 76.0f / 255.0f, col.R, 0.01f);
  ASSERT_NEAR(1.42 * 124.0f / 255.0f, col.G, 0.01f);
  ASSERT_NEAR(1.42 * 138.0f / 255.0f, col.B, 0.01f);

  GlColor col1 = GlColor::FromHSV(radians(193), 45.0f / 100.f, 54.0f / 100.0f);
  col1.darker();
  ASSERT_NEAR(0.7 * 76.0f / 255.0f, col1.R, 0.01f);
  ASSERT_NEAR(0.7 * 124.0f / 255.0f, col1.G, 0.01f);
  ASSERT_NEAR(0.7 * 138.0f / 255.0f, col1.B, 0.01f);
}
}

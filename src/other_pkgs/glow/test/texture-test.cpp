#include <gtest/gtest.h>

#include <glow/GlTexture.h>
#include <glow/GlState.h>
#include <glow/GlTextureRectangle.h>

using namespace glow;

// TODO: write tests for resize, etc.
// TODO: test side effects & pre and post conditions, i.e., GlState::queryAll() should be essentially the same before
// and after copy.

TEST(TextureTest, loadTexture) {
}

TEST(TextureTest, assignTest) {
  GlTexture texture(100, 50, TextureFormat::RGBA_FLOAT);
  ASSERT_NO_THROW(CheckGlError());
  std::vector<float> img(100 * 50 * 4);
  ASSERT_EQ(static_cast<size_t>(100 * 50 * 4), img.size());
  for (uint32_t i = 0; i < img.size(); ++i) {
    img[i] = 1.45f * i;
  }
  texture.assign(PixelFormat::RGBA, PixelType::FLOAT, &img[0]);
  ASSERT_NO_THROW(CheckGlError());

  std::vector<float> device_mem;
  texture.download(device_mem);
  ASSERT_EQ(img.size(), device_mem.size());
  for (uint32_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], device_mem[i]);
  }
}

TEST(TextureTest, copyTest) {
  GlTexture texture(100, 50, TextureFormat::RGBA_FLOAT);
  ASSERT_NO_THROW(CheckGlError());

  std::vector<float> img(100 * 50 * 4);
  for (uint32_t i = 0; i < img.size(); ++i) {
    img[i] = 1.45f * i;
  }

  texture.assign(PixelFormat::RGBA, PixelType::FLOAT, &img[0]);
  ASSERT_NO_THROW(CheckGlError());

  GlTexture texture2(100, 50, TextureFormat::RGBA_FLOAT);

  GlState state_before = GlState::queryAll();

  texture2.copy(texture);

  GlState state_afterwards = GlState::queryAll();

  if (state_before != state_afterwards) {
    state_before.difference(state_afterwards);
  }

  ASSERT_TRUE(state_before == state_afterwards);

  std::vector<float> device_mem;
  texture2.download(device_mem);
  ASSERT_EQ(img.size(), device_mem.size());
  //  for (uint32_t i = 0; i < 12; ++i)
  //  {
  //    std::cout << device_mem[i] << ", ";
  //  }
  //  std::cout << std::endl;

  for (uint32_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], device_mem[i]);
  }
}

TEST(TextureTest, cloneTest) {
  GlTexture texture(100, 50, TextureFormat::RGBA_FLOAT);
  ASSERT_NO_THROW(CheckGlError());

  std::vector<float> img(100 * 50 * 4);
  for (uint32_t i = 0; i < img.size(); ++i) {
    img[i] = 1234.3 * i;
  }

  texture.assign(PixelFormat::RGBA, PixelType::FLOAT, &img[0]);
  ASSERT_NO_THROW(CheckGlError());

  GlTexture clone = texture.clone();

  ASSERT_EQ(texture.width(), clone.width());
  ASSERT_EQ(texture.height(), clone.height());

  std::vector<float> device_mem;
  clone.download(device_mem);
  ASSERT_EQ(img.size(), device_mem.size());
  //  for (uint32_t i = 0; i < 12; ++i)
  //  {
  //    std::cout << device_mem[i] << ", ";
  //  }
  //  std::cout << std::endl;

  for (uint32_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], device_mem[i]);
  }
}

TEST(TextureRectangleTest, loadTexture) {
}

TEST(TextureRectangleTest, assignTest) {
  GlTextureRectangle texture(100, 50, TextureFormat::RGBA_FLOAT);
  ASSERT_NO_THROW(CheckGlError());
  std::vector<float> img(100 * 50 * 4);
  ASSERT_EQ(static_cast<size_t>(100 * 50 * 4), img.size());
  for (uint32_t i = 0; i < img.size(); ++i) {
    img[i] = 1.45f * i;
  }
  texture.assign(PixelFormat::RGBA, PixelType::FLOAT, &img[0]);
  ASSERT_NO_THROW(CheckGlError());

  std::vector<float> device_mem;
  texture.download(device_mem);
  ASSERT_EQ(img.size(), device_mem.size());
  for (uint32_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], device_mem[i]);
  }
}

TEST(TextureRectangleTest, copyTest) {
  GlTextureRectangle texture(100, 50, TextureFormat::RGBA_FLOAT);
  ASSERT_NO_THROW(CheckGlError());

  std::vector<float> img(100 * 50 * 4);
  for (uint32_t i = 0; i < img.size(); ++i) {
    img[i] = 1.45f * i;
  }

  texture.assign(PixelFormat::RGBA, PixelType::FLOAT, &img[0]);
  ASSERT_NO_THROW(CheckGlError());

  GlTextureRectangle texture2(100, 50, TextureFormat::RGBA_FLOAT);
  texture2.copy(texture);

  std::vector<float> device_mem;
  texture2.download(device_mem);

  //  for (uint32_t i = 0; i < 12; ++i)
  //  {
  //    std::cout << device_mem[i] << ", ";
  //  }
  //  std::cout << std::endl;

  ASSERT_EQ(img.size(), device_mem.size());
  //  for (uint32_t i = 0; i < 12; ++i)
  //  {
  //    std::cout << device_mem[i] << ", ";
  //  }
  //  std::cout << std::endl;

  for (uint32_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], device_mem[i]);
  }
}

TEST(TextureRectangleTest, copyTextureTest) {
  GlTexture texture(100, 50, TextureFormat::RGBA_FLOAT);
  ASSERT_NO_THROW(CheckGlError());

  std::vector<float> img(100 * 50 * 4);
  for (uint32_t i = 0; i < img.size(); ++i) {
    img[i] = 1.45f * i;
  }

  texture.assign(PixelFormat::RGBA, PixelType::FLOAT, &img[0]);
  ASSERT_NO_THROW(CheckGlError());

  GlTextureRectangle texture2(100, 50, TextureFormat::RGBA_FLOAT);
  texture2.copy(texture);

  std::vector<float> device_mem;
  texture2.download(device_mem);

  //  for (uint32_t i = 0; i < 12; ++i)
  //  {
  //    std::cout << device_mem[i] << ", ";
  //  }
  //  std::cout << std::endl;

  ASSERT_EQ(img.size(), device_mem.size());
  //  for (uint32_t i = 0; i < 12; ++i)
  //  {
  //    std::cout << device_mem[i] << ", ";
  //  }
  //  std::cout << std::endl;

  for (uint32_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], device_mem[i]);
  }
}

TEST(TextureRectangleTest, cloneTest) {
  GlTextureRectangle texture(100, 50, TextureFormat::RGBA_FLOAT);
  ASSERT_NO_THROW(CheckGlError());

  std::vector<float> img(100 * 50 * 4);
  for (uint32_t i = 0; i < img.size(); ++i) {
    img[i] = 1234.3 * i;
  }

  texture.assign(PixelFormat::RGBA, PixelType::FLOAT, &img[0]);
  ASSERT_NO_THROW(CheckGlError());

  GlTextureRectangle clone = texture.clone();

  ASSERT_EQ(texture.width(), clone.width());
  ASSERT_EQ(texture.height(), clone.height());

  std::vector<float> device_mem;
  clone.download(device_mem);
  ASSERT_EQ(img.size(), device_mem.size());
  //  for (uint32_t i = 0; i < 12; ++i)
  //  {
  //    std::cout << device_mem[i] << ", ";
  //  }
  //  std::cout << std::endl;

  for (uint32_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], device_mem[i]);
  }
}

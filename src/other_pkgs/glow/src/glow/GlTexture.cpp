#include "GlTexture.h"

#include <stdint.h>
#include <cassert>
#include <vector>

#include "GlFramebuffer.h"
#include "GlProgram.h"
#include "GlSampler.h"
#include "GlState.h"
#include "GlVertexArray.h"
#include "glutil.h"

#include <fstream>

#include <boost/filesystem.hpp>

namespace glow {

GLuint GlTexture::boundTexture_ = 0;

template <>
void GlTexture::download<float>(std::vector<float>& data) const {
  data.resize(numComponents(format_) * width_ * height_);
  GLuint id = bindTransparently();

  GLenum pixFormat = GL_RGBA;

  switch (format_) {
    case TextureFormat::R:
    case TextureFormat::R_FLOAT:
      pixFormat = GL_RED;
      break;

    case TextureFormat::R_INTEGER:
      pixFormat = GL_RED_INTEGER;
      break;

    case TextureFormat::RG:
    case TextureFormat::RG_FLOAT:
      pixFormat = GL_RG;
      break;

    case TextureFormat::RG_INTEGER:
      pixFormat = GL_RG_INTEGER;
      break;

    case TextureFormat::RGB:
    case TextureFormat::RGB_FLOAT:
      pixFormat = GL_RGB;
      break;

    case TextureFormat::RGB_INTEGER:
      pixFormat = GL_RGB_INTEGER;
      break;

    case TextureFormat::RGBA:
    case TextureFormat::RGBA_FLOAT:
      pixFormat = GL_RGBA;
      break;

    case TextureFormat::RGBA_INTEGER:
      pixFormat = GL_RGBA_INTEGER;
      break;

    case TextureFormat::DEPTH:
      pixFormat = GL_DEPTH;
      break;

    case TextureFormat::DEPTH_STENCIL:
      pixFormat = GL_DEPTH_STENCIL;
      break;
  }

  glGetTexImage(target_, 0, pixFormat, GL_FLOAT, reinterpret_cast<GLvoid*>(&data[0]));
  CheckGlError();
  releaseTransparently(id);
}

template <>
void GlTexture::download<vec4>(std::vector<vec4>& data) const {
  data.resize(width_ * height_);
  GLuint id = bindTransparently();
  if (format_ == TextureFormat::R_INTEGER)  // || format_ == TextureFormat::RG_INTEGER|| format_ ==
                                            // TextureFormat::RGB_INTEGER || format_ == TextureFormat::RGBA_INTEGER)
  {
    std::vector<int32_t> d(width_ * height_);

    glGetTexImage(target_, 0, GL_RED_INTEGER, GL_INT, reinterpret_cast<GLvoid*>(&d[0]));
    for (uint32_t i = 0; i < width_ * height_; ++i) {
      data[i].x = d[i];
      data[i].y = 0;
      data[i].z = 0;
      data[i].w = 0;
    }
  } else {
    glGetTexImage(target_, 0, GL_RGBA, GL_FLOAT, reinterpret_cast<GLvoid*>(&data[0]));
  }

  CheckGlError();
  releaseTransparently(id);
}

template <>
void GlTexture::download(Eigen::Vector4f* ptr) const {
  GLuint id = bindTransparently();
  glGetTexImage(target_, 0, GL_RGBA, GL_FLOAT, reinterpret_cast<GLvoid*>(ptr));
  CheckGlError();
  releaseTransparently(id);
}

template <>
void GlTexture::download(PixelFormat pixelfmt, float* ptr) const {
  GLuint id = bindTransparently();
  glGetTexImage(target_, 0, static_cast<GLenum>(pixelfmt), GL_FLOAT, reinterpret_cast<GLvoid*>(ptr));
  releaseTransparently(id);
}

template <>
void GlTexture::download(PixelFormat pixelfmt, int32_t* ptr) const {
  std::cout << "downloading int." << std::endl;
  GLuint id = bindTransparently();
  glGetTexImage(target_, 0, static_cast<GLenum>(pixelfmt), GL_INT, reinterpret_cast<GLvoid*>(ptr));
  releaseTransparently(id);
  CheckGlError();
  std::cout << "finished." << std::endl;
}

GlTexture::GlTexture(uint32_t width, TextureFormat format) : width_(width), height_(0), depth_(0), format_(format) {
  glGenTextures(1, &id_);
  target_ = GL_TEXTURE_1D;
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteTextures(1, ptr);
    delete ptr;
  });

  // allocate space.
  GLuint old_id = bindTransparently();
  allocateMemory();
  glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(target_, GL_TEXTURE_WRAP_S, GL_REPEAT);
  releaseTransparently(old_id);

  CheckGlError();
}

GlTexture::GlTexture(uint32_t width, uint32_t height, TextureFormat format)
    : width_(width), height_(height), depth_(0), format_(format) {
  glGenTextures(1, &id_);
  target_ = GL_TEXTURE_2D;
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteTextures(1, ptr);
    delete ptr;
  });

  // allocate space.
  GLuint old_id = bindTransparently();
  allocateMemory();
  glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(target_, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(target_, GL_TEXTURE_WRAP_T, GL_REPEAT);
  releaseTransparently(old_id);

  CheckGlError();
}

GlTexture::GlTexture(uint32_t width, uint32_t height, uint32_t depth, TextureFormat format)
    : width_(width), height_(height), depth_(depth), format_(format) {
  glGenTextures(1, &id_);
  target_ = GL_TEXTURE_3D;
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteTextures(1, ptr);
    delete ptr;
  });

  // allocate space.
  GLuint old_id = bindTransparently();
  allocateMemory();
  glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(target_, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(target_, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(target_, GL_TEXTURE_WRAP_R, GL_REPEAT);
  releaseTransparently(old_id);

  CheckGlError();
}

GlTexture::~GlTexture() {
  if (boundTexture_ == id_) {
    release();
    assert((boundTexture_ != id_) && "Texture object still bound.");
  }
}

void GlTexture::copy(const GlTexture& other) {
  if (height_ == 0 || depth_ > 0) {
    // FIXME: implement.
    std::cout << "WARNING: copy of 1D and 3D textures not supported." << std::endl;
    return;  // copy only data for two-dimensional textures.
  }

  GLint ov[4];
  GLboolean depthTest;
  GLint old_fbo;

  glGetIntegerv(GL_VIEWPORT, ov);
  glGetBooleanv(GL_DEPTH_TEST, &depthTest);
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_fbo);

  GlVertexArray vao;

  // create a framebuffer with destination texture and use fragment shader for copy.
  GlSampler sampler;
  sampler.setMinifyingOperation(TexMinOp::NEAREST);
  sampler.setMagnifyingOperation(TexMagOp::NEAREST);
  sampler.setWrapOperation(TexWrapOp::REPEAT, TexWrapOp::REPEAT);

  GlFramebuffer copy_fbo(width_, height_, FramebufferTarget::DRAW);
  GlRenderbuffer rbo(width_, height_, RenderbufferFormat::DEPTH_STENCIL);
  copy_fbo.attach(FramebufferAttachment::COLOR0, *this);
  copy_fbo.attach(FramebufferAttachment::DEPTH_STENCIL, rbo);
  assert(copy_fbo.valid());

  std::string empty_vert = "#version 330 core\nvoid main(){}";
  std::string quad_geom = "#version 330 core\nlayout(points) in;\nlayout(triangle_strip, max_vertices = 4) out;\n";
  quad_geom += "out vec2 texCoords;\nvoid main(){\n";
  quad_geom += "  gl_Position = vec4(1.0, 1.0, 0.0, 1.0);\ntexCoords = vec2(1.0, 1.0);\nEmitVertex();\n";
  quad_geom += "  gl_Position = vec4(-1.0, 1.0, 0.0, 1.0);\ntexCoords = vec2(0.0, 1.0);\nEmitVertex();\n";
  quad_geom += "  gl_Position = vec4(1.0,-1.0, 0.0, 1.0);\ntexCoords = vec2(1.0, 0.0);\nEmitVertex();\n";
  quad_geom += "  gl_Position = vec4(-1.0,-1.0, 0.0, 1.0);\ntexCoords = vec2(0.0, 0.0);\nEmitVertex();\n";
  quad_geom += "  EndPrimitive();\n}";
  std::string copy_frag = "#version 330 core\nin vec2 texCoords;\n";

  switch (other.format_) {
    case TextureFormat::R_INTEGER:
    case TextureFormat::RG_INTEGER:
    case TextureFormat::RGB_INTEGER:
    case TextureFormat::RGBA_INTEGER:
      copy_frag += "uniform isampler2D tex_other;\n";
      break;
    default:
      copy_frag += "uniform sampler2D tex_other;\n";
  }
  // do we need to consider input/ouptut formats?
  copy_frag += "out vec4 color;\nvoid main(){\n  color = texture(tex_other, texCoords); \n}";

  GlProgram copy_program;
  copy_program.attach(GlShader(ShaderType::VERTEX_SHADER, empty_vert));
  copy_program.attach(GlShader(ShaderType::GEOMETRY_SHADER, quad_geom));
  copy_program.attach(GlShader(ShaderType::FRAGMENT_SHADER, copy_frag));
  copy_program.link();
  copy_program.setUniform(GlUniform<int32_t>("tex_other", 0));

  copy_fbo.bind();
  copy_program.bind();
  vao.bind();

  glDisable(GL_DEPTH_TEST);
  glViewport(0, 0, width_, height_);  // set viewport to new values

  glActiveTexture(GL_TEXTURE0);
  uint32_t old_id = other.bindTransparently();
  sampler.bind(0);

  glDrawArrays(GL_POINTS, 0, 1);

  sampler.release(0);
  vao.release();
  other.releaseTransparently(old_id);
  copy_program.release();

  glEnable(GL_DEPTH_TEST);
  copy_fbo.release();

  glFinish();

  // restore settings.
  glViewport(ov[0], ov[1], ov[2], ov[3]);
  if (depthTest)
    glEnable(GL_DEPTH_TEST);
  else
    glDisable(GL_DEPTH_TEST);

  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_fbo);
}

GlTexture GlTexture::clone() const {
  if (height_ > 0 && depth_ > 0) {
    GlTexture tex(width_, height_, depth_, format_);
    tex.copy(*this);

    return tex;
  } else if (height_ > 0) {
    GlTexture tex(width_, height_, format_);
    tex.copy(*this);

    return tex;
  } else {
    GlTexture tex(width_, format_);
    tex.copy(*this);

    return tex;
  }

  throw GlTextureError("invalid texture.");
}

void GlTexture::bind() {
  glBindTexture(target_, id_);
  boundTexture_ = id_;
}

void GlTexture::release() {
  glBindTexture(target_, 0);
  boundTexture_ = 0;
}

void GlTexture::setMinifyingOperation(TexMinOp minifyingOperation) {
  GLuint old_id = bindTransparently();
  glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, static_cast<GLenum>(minifyingOperation));
  releaseTransparently(old_id);
}

void GlTexture::setMagnifyingOperation(TexMagOp magnifyingOperation) {
  GLuint old_id = bindTransparently();
  glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, static_cast<GLenum>(magnifyingOperation));
  releaseTransparently(old_id);
}

void GlTexture::setWrapOperation(TexWrapOp wrap_s) {
  GLuint old_id = bindTransparently();
  glTexParameteri(target_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
  releaseTransparently(old_id);
}

void GlTexture::setWrapOperation(TexWrapOp wrap_s, TexWrapOp wrap_t) {
  GLuint old_id = bindTransparently();
  glTexParameteri(target_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
  glTexParameteri(target_, GL_TEXTURE_WRAP_T, static_cast<GLenum>(wrap_t));
  releaseTransparently(old_id);
}

void GlTexture::setWrapOperation(TexWrapOp wrap_s, TexWrapOp wrap_t, TexWrapOp wrap_r) {
  GLuint old_id = bindTransparently();
  glTexParameteri(target_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
  glTexParameteri(target_, GL_TEXTURE_WRAP_T, static_cast<GLenum>(wrap_t));
  glTexParameteri(target_, GL_TEXTURE_WRAP_R, static_cast<GLenum>(wrap_r));
  releaseTransparently(old_id);
}

void GlTexture::setTextureSwizzle(TexSwizzle red, TexSwizzle green, TexSwizzle blue, TexSwizzle alpha) {
  GLint swizzleMask[] = {static_cast<GLint>(red), static_cast<GLint>(green), static_cast<GLint>(blue),
                         static_cast<GLint>(alpha)};

  GLuint old_id = bindTransparently();
  glTexParameteriv(target_, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
  releaseTransparently(old_id);
}

bool GlTexture::save(const std::string& filename) const {
  // TODO: implement saving to image. (libpng, etc.)
  // note: texture transfer is always specified by pixel transfer; therefore we can have arbitrary output,...

  std::string ext = glow::extension(filename);
  if (ext == ".ppm") {
    if (height_ == 0) {
      std::ofstream out(filename.c_str());
      out << "P6" << std::endl;
      out << width_ << std::endl;
      out << 1 << std::endl;
      out << "255" << std::endl;

      std::vector<float> colors(width_ * 3);
      GLuint id = bindTransparently();
      glGetTexImage(target_, 0, GL_RGB, GL_FLOAT, reinterpret_cast<GLvoid*>(&colors[0]));
      CheckGlError();
      releaseTransparently(id);

      std::vector<char> data(width_ * 3);

      for (uint32_t x = 0; x < width_; ++x) {
        int32_t r = 255 * colors[3 * x];
        int32_t g = 255 * colors[3 * x + 1];
        int32_t b = 255 * colors[3 * x + 2];
        data[3 * x] = (char)std::max(std::min(255, r), 0);
        data[3 * x + 1] = (char)std::max(std::min(255, g), 0);
        data[3 * x + 2] = (char)std::max(std::min(255, b), 0);
      }

      out.write(&data[0], height_ * 3);

      out.close();
      return true;

    } else {
      std::ofstream out(filename.c_str());
      out << "P6" << std::endl;
      out << width_ << std::endl;
      out << height_ << std::endl;
      out << "255" << std::endl;

      std::vector<float> colors(width_ * height_ * 3);
      GLuint id = bindTransparently();
      glGetTexImage(target_, 0, GL_RGB, GL_FLOAT, reinterpret_cast<GLvoid*>(&colors[0]));
      CheckGlError();
      releaseTransparently(id);

      std::vector<char> data(width_ * height_ * 3);

      for (uint32_t x = 0; x < width_; ++x) {
        for (uint32_t y = 0; y < height_; ++y) {
          int32_t r = 255 * colors[3 * (x + (height_ - 1 - y) * width_)];
          int32_t g = 255 * colors[3 * (x + (height_ - 1 - y) * width_) + 1];
          int32_t b = 255 * colors[3 * (x + (height_ - 1 - y) * width_) + 2];
          data[3 * (x + y * width_)] = (char)std::max(std::min(255, r), 0);
          data[3 * (x + y * width_) + 1] = (char)std::max(std::min(255, g), 0);
          data[3 * (x + y * width_) + 2] = (char)std::max(std::min(255, b), 0);
        }
      }

      out.write(&data[0], width_ * height_ * 3);

      out.close();
      return true;
    }
  }

  return false;
}

GlTexture GlTexture::loadTexture(const std::string& filename) {
  assert(false && "Loading of textures not implemented.");
  // TODO: implement loading to image. (libpng, etc.)
  return GlTexture(0);
}

GLuint GlTexture::bindTransparently() const {
  if (boundTexture_ == id_) return id_;

  glBindTexture(target_, id_);

  return boundTexture_;
}

void GlTexture::releaseTransparently(GLuint old_id) const {
  if (old_id == id_) return;

  glBindTexture(target_, old_id);
}

void GlTexture::resize(uint32_t width) {
  if (height_ > 0 || depth_ > 0) {
    throw GlTextureError("Texture not one-dimensional. Hence, resize needs more then just one dimension.");
  }

  width_ = width;
  GLuint id = bindTransparently();
  allocateMemory();
  releaseTransparently(id);
}

void GlTexture::resize(uint32_t width, uint32_t height) {
  if (height_ == 0 || depth_ > 0) {
    throw GlTextureError("Texture not two-dimensional. Hence, resize needs more or less dimension.");
  }

  width_ = width;
  height_ = height;
  GLuint id = bindTransparently();
  allocateMemory();
  releaseTransparently(id);
}

void GlTexture::resize(uint32_t width, uint32_t height, uint32_t depth) {
  if (height_ == 0 || depth_ == 0) {
    throw GlTextureError("Texture not three-dimensional. Hence, resize needs more or less dimension.");
  }

  width_ = width;
  height_ = height;
  depth_ = depth;

  GLuint id = bindTransparently();
  allocateMemory();
  releaseTransparently(id);
}

uint32_t GlTexture::width() const {
  return width_;
}

uint32_t GlTexture::height() const {
  return height_;
}

uint32_t GlTexture::depth() const {
  return depth_;
}

uint32_t GlTexture::numComponents(TextureFormat fmt) {
  switch (fmt) {
    case TextureFormat::R:
    case TextureFormat::R_INTEGER:
    case TextureFormat::R_FLOAT:
      return 1;
    case TextureFormat::RG:
    case TextureFormat::RG_INTEGER:
    case TextureFormat::RG_FLOAT:
      return 2;
    case TextureFormat::RGB:
    case TextureFormat::RGB_INTEGER:
    case TextureFormat::RGB_FLOAT:
      return 3;
    case TextureFormat::RGBA:
    case TextureFormat::RGBA_INTEGER:
    case TextureFormat::RGBA_FLOAT:
      return 4;
    //    case TextureFormat::GRAY:
    //      return 1;
    //    case TextureFormat::INTENSITY:
    //      return 1;
    case TextureFormat::DEPTH:
      return 1;
    case TextureFormat::DEPTH_STENCIL:
      return 2;
  }

  return 4;
}

void writeBitmap(const std::string& filename, const unsigned char* data) {}

void GlTexture::allocateMemory() {
  CheckGlError();
  if (width_ == 0 && height_ == 0 && depth_ == 0) return;

  // select pix format and type according to texture format.
  GLint texFormat = static_cast<GLint>(format_);
  GLenum pixFormat = GL_RGBA;
  GLenum pixType = GL_UNSIGNED_BYTE;

  switch (format_) {
    case TextureFormat::R:
    case TextureFormat::RG:
    case TextureFormat::RGB:
    case TextureFormat::RGBA:
      pixType = GL_UNSIGNED_BYTE;
      break;

    case TextureFormat::R_INTEGER:
    case TextureFormat::RG_INTEGER:
    case TextureFormat::RGB_INTEGER:
    case TextureFormat::RGBA_INTEGER:
      pixType = GL_INT;
      break;

    case TextureFormat::R_FLOAT:
    case TextureFormat::RG_FLOAT:
    case TextureFormat::RGB_FLOAT:
    case TextureFormat::RGBA_FLOAT:
      pixType = GL_FLOAT;
      break;

    case TextureFormat::DEPTH:
      pixType = GL_UNSIGNED_BYTE;  // ??
      break;

    case TextureFormat::DEPTH_STENCIL:
      pixType = GL_DEPTH24_STENCIL8;
      break;
  }

  switch (format_) {
    case TextureFormat::R:
    case TextureFormat::R_FLOAT:
      pixFormat = GL_RED;
      break;

    case TextureFormat::R_INTEGER:
      pixFormat = GL_RED_INTEGER;
      break;

    case TextureFormat::RG:
    case TextureFormat::RG_FLOAT:
      pixFormat = GL_RG;
      break;

    case TextureFormat::RG_INTEGER:
      pixFormat = GL_RG_INTEGER;
      break;

    case TextureFormat::RGB:
    case TextureFormat::RGB_FLOAT:
      pixFormat = GL_RGB;
      break;

    case TextureFormat::RGB_INTEGER:
      pixFormat = GL_RGB_INTEGER;
      break;

    case TextureFormat::RGBA:
    case TextureFormat::RGBA_FLOAT:
      pixFormat = GL_RGBA;
      break;

    case TextureFormat::RGBA_INTEGER:
      pixFormat = GL_RGBA_INTEGER;
      break;

    case TextureFormat::DEPTH:
      pixFormat = GL_DEPTH;
      break;

    case TextureFormat::DEPTH_STENCIL:
      pixFormat = GL_DEPTH_STENCIL;
      break;
  }

  // this ensures that integral internal formats are matched to integral pixel formats:
  if (height_ == 0 && depth_ == 0)
    glTexImage1D(target_, 0, texFormat, width_, 0, pixFormat, pixType, nullptr);
  else if (depth_ == 0)
    glTexImage2D(target_, 0, texFormat, width_, height_, 0, pixFormat, pixType, nullptr);
  else
    glTexImage3D(target_, 0, texFormat, width_, height_, depth_, 0, pixFormat, pixType, nullptr);

  CheckGlError();
}

void GlTexture::generateMipmaps() {
  GLuint id = bindTransparently();
  glGenerateMipmap(target_);
  releaseTransparently(id);
}
}
/* namespace rv */

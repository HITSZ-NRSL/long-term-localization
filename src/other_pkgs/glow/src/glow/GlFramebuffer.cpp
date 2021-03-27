/*
 * GlFramebuffer.cpp
 *
 *  Created on: Apr 28, 2016
 *      Author: behley
 */

#include "GlFramebuffer.h"
#include "glexception.h"

#include <sstream>

namespace glow {

GLuint GlFramebuffer::boundFramebuffer_ = 0;

GlFramebuffer::GlFramebuffer(uint32_t width, uint32_t height, FramebufferTarget target)
    : target_(static_cast<GLenum>(target)), width_(width), height_(height) {
  glGenFramebuffers(1, &id_);
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteFramebuffers(1, ptr);
    delete ptr;
  });

  CheckGlError();
}

GlFramebuffer::~GlFramebuffer() {
  if (boundFramebuffer_ == id_) {
    release();
    assert((boundFramebuffer_ != id_) && "Framebuffer object still bound.");
  }
  attachments_.clear();
}

inline std::string error_string(GLint id) {
  switch (id) {
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
      return "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
      return "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
      return "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
      return "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
    case GL_FRAMEBUFFER_UNSUPPORTED:
      return "GL_FRAMEBUFFER_UNSUPPORTED";
  }

  return "UNKNOWN ERROR.";
}

void GlFramebuffer::bind() {
  if (!valid_) {
    std::stringstream reason;
    reason << "Invalid framebuffer object. Code ";
    glBindFramebuffer(target_, id_);
    GLint errid = glCheckFramebufferStatus(target_);
    reason << errid << "(" << error_string(errid) << ")";
    glBindFramebuffer(target_, 0);
    throw GlFramebufferError(reason.str());
  }

  boundFramebuffer_ = id_;
  glBindFramebuffer(target_, id_);
}

void GlFramebuffer::release() {
  boundFramebuffer_ = 0;
  glBindFramebuffer(target_, 0);
}

GLuint GlFramebuffer::bindTransparently() {
  glBindFramebuffer(target_, id_);

  return boundFramebuffer_;
}

void GlFramebuffer::releaseTransparently(GLuint old_id) {
  glBindFramebuffer(target_, old_id);
}

void GlFramebuffer::attach(FramebufferAttachment target, GlTexture& texture) {
  if (texture.target_ != GL_TEXTURE_2D) throw GlFramebufferError("Expected two-dimensional texture");
  if (texture.width() < width_) {
    std::stringstream error;
    error << "Texture's width should be at least " << width_;
    throw GlFramebufferError(error.str());
  }

  if (texture.height() < height_) {
    std::stringstream error;
    error << "Texture's height should be at least " << height_;
    throw GlFramebufferError(error.str());
  }

  GLuint old_buffer = bindTransparently();

  glFramebufferTexture2D(target_, static_cast<GLenum>(target), texture.target_, texture.id(), 0);

  attachments_[target] = texture.ptr_;  // get pointer to prevent deallocation before framebuffer is deallocated.
  valid_ = (glCheckFramebufferStatus(target_) == GL_FRAMEBUFFER_COMPLETE);

  releaseTransparently(old_buffer);
}

/** \brief attach rectangular texture to given target.
 *
 *  The texture must have at least the width and height of the framebuffer object.
 **/
void GlFramebuffer::attach(FramebufferAttachment target, GlTextureRectangle& texture) {
  if (texture.width() < width_) {
    std::stringstream error;
    error << "Texture's width should be at least " << width_;
    throw GlFramebufferError(error.str());
  }

  if (texture.height() < height_) {
    std::stringstream error;
    error << "Texture's height should be at least " << height_;
    throw GlFramebufferError(error.str());
  }

  GLuint old_buffer = bindTransparently();

  glFramebufferTexture2D(target_, static_cast<GLenum>(target), GL_TEXTURE_RECTANGLE, texture.id(), 0);

  attachments_[target] = texture.ptr_;  // get pointer to prevent deallocation before framebuffer is deallocated.
  valid_ = (glCheckFramebufferStatus(target_) == GL_FRAMEBUFFER_COMPLETE);

  releaseTransparently(old_buffer);
}

void GlFramebuffer::attach(FramebufferAttachment target, GlRenderbuffer& buffer) {
  if (buffer.width() < width_) {
    std::stringstream error;
    error << "Renderbuffer's width should be at least " << width_;
    throw GlFramebufferError(error.str());
  }

  if (buffer.height() < height_) {
    std::stringstream error;
    error << "Renderbuffer's width should be at least " << height_;
    throw GlFramebufferError(error.str());
  }

  GLuint old_buffer = bindTransparently();

  glFramebufferRenderbuffer(target_, static_cast<GLenum>(target), GL_RENDERBUFFER, buffer.id());

  attachments_[target] = buffer.ptr_;  // get pointer to prevent deallocation before framebuffer is deallocated.
  valid_ = (glCheckFramebufferStatus(target_) == GL_FRAMEBUFFER_COMPLETE);

  releaseTransparently(old_buffer);
}

bool GlFramebuffer::valid() const {
  return valid_;
}

uint32_t GlFramebuffer::width() const {
  return width_;
}

uint32_t GlFramebuffer::height() const {
  return height_;
}

void GlFramebuffer::resize(uint32_t width, uint32_t height) {
  valid_ = false;
  width_ = width;
  height_ = height;
}

} /* namespace rv */

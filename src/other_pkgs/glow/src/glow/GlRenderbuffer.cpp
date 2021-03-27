#include "GlRenderbuffer.h"

namespace glow {

GlRenderbuffer::GlRenderbuffer(uint32_t width, uint32_t height, RenderbufferFormat fmt)
    : format_(static_cast<GLenum>(fmt)), width_(width), height_(height) {
  glGenRenderbuffers(1, &id_);
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteRenderbuffers(1, ptr);
    delete ptr;
  });

  // allocate storage.
  glBindRenderbuffer(GL_RENDERBUFFER, id_);
  glRenderbufferStorage(GL_RENDERBUFFER, static_cast<GLenum>(fmt), width_, height_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  CheckGlError();
}

void GlRenderbuffer::bind() {
  glBindRenderbuffer(GL_RENDERBUFFER, id_);
}

void GlRenderbuffer::release() {
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
}

uint32_t GlRenderbuffer::width() const {
  return width_;
}

uint32_t GlRenderbuffer::height() const {
  return height_;
}

} /* namespace rv */

#include "GlVertexArray.h"
#include "glexception.h"
#include <cassert>

namespace glow {

GLuint GlVertexArray::boundVAO_ = 0;

GlVertexArray::GlVertexArray() {
  glGenVertexArrays(1, &id_);
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteVertexArrays(1, ptr);
    delete ptr;
  });
}

GlVertexArray::~GlVertexArray() {
}

void GlVertexArray::bind() {
  assert((boundVAO_ == 0 || boundVAO_ == id_) && "Other vertex array object still active?");
  boundVAO_ = id_;
  glBindVertexArray(id_);
}

void GlVertexArray::release() {
  assert(boundVAO_ == id_ && "Different vertex array object bound in between?");
  boundVAO_ = 0;
  glBindVertexArray(0);
}

void GlVertexArray::enableVertexAttribute(uint32_t idx) {
  GLuint oldvao = bindTransparently();
  glEnableVertexAttribArray(static_cast<GLuint>(idx));
  releaseTransparently(oldvao);
}

void GlVertexArray::disableVertexAttribute(uint32_t idx) {
  GLuint oldvao = bindTransparently();
  glDisableVertexAttribArray(static_cast<GLuint>(idx));
  releaseTransparently(oldvao);
}

GLuint GlVertexArray::bindTransparently() {
  if (boundVAO_ == id_) return id_;

  glBindVertexArray(id_);

  return boundVAO_;
}

void GlVertexArray::releaseTransparently(GLuint old_vao) {
  if (old_vao == id_) return;  // nothing changed.

  glBindVertexArray(old_vao);
}

} /* namespace rv */

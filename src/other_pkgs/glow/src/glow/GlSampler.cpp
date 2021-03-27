#include "GlSampler.h"

namespace glow {

GlSampler::GlSampler() {
  glGenSamplers(1, &id_);
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteSamplers(1, ptr);
    delete ptr;
  });
}

/** \brief use sampler for specific texture unit identified by its index (0, 1, ...). **/
void GlSampler::bind(uint32_t textureUnitId) {
  glBindSampler(static_cast<GLuint>(textureUnitId), id_);
}

/** \brief "unuse" sampler for given texture unit.  **/
void GlSampler::release(uint32_t textureUnitId) {
  glBindSampler(static_cast<GLuint>(textureUnitId), 0);
}

void GlSampler::bind() {
}

void GlSampler::release() {
}

void GlSampler::setMinifyingOperation(TexMinOp minifyingOperation) {
  glSamplerParameteri(id_, GL_TEXTURE_MIN_FILTER, static_cast<GLenum>(minifyingOperation));
}

void GlSampler::setMagnifyingOperation(TexMagOp magnifyingOperation) {
  glSamplerParameteri(id_, GL_TEXTURE_MAG_FILTER, static_cast<GLenum>(magnifyingOperation));
}

void GlSampler::setWrapOperation(TexWrapOp wrap_s) {
  glSamplerParameteri(id_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
}

void GlSampler::setWrapOperation(TexWrapOp wrap_s, TexWrapOp wrap_t) {
  glSamplerParameteri(id_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
  glSamplerParameteri(id_, GL_TEXTURE_WRAP_T, static_cast<GLenum>(wrap_t));
}

void GlSampler::setWrapOperation(TexWrapOp wrap_s, TexWrapOp wrap_t, TexWrapOp wrap_r) {
  glSamplerParameteri(id_, GL_TEXTURE_WRAP_S, static_cast<GLenum>(wrap_s));
  glSamplerParameteri(id_, GL_TEXTURE_WRAP_T, static_cast<GLenum>(wrap_t));
  glSamplerParameteri(id_, GL_TEXTURE_WRAP_R, static_cast<GLenum>(wrap_r));
}

} /* namespace rv */

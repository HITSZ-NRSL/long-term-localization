#include "GlUniform.h"
#include "glexception.h"
#include "glutil.h"

namespace glow {

// explicit definition of uniforms for some common types.

// Common Eigen types:
template <>
void GlUniform<Eigen::Matrix4f>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniformMatrix4fv(loc, 1, GL_FALSE, data_.data());
}

template <>
void GlUniform<Eigen::Vector4f>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform4fv(loc, 1, data_.data());
}

template <>
void GlUniform<Eigen::Vector3f>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform3fv(loc, 1, data_.data());
}

template <>
void GlUniform<vec4>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform4fv(loc, 1, &data_.x);
}

template <>
void GlUniform<vec3>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform3fv(loc, 1, &data_.x);
}

template <>
void GlUniform<vec2>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform2fv(loc, 1, &data_.x);
}

// geometry.h types.

// some primitive types
template <>
void GlUniform<int32_t>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform1i(loc, static_cast<GLint>(data_));
}

template <>
void GlUniform<uint32_t>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform1ui(loc, static_cast<GLuint>(data_));
}

template <>
void GlUniform<bool>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unkown or unused in program.");
  glUniform1i(loc, static_cast<GLint>(data_));
}

template <>
void GlUniform<float>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform1f(loc, static_cast<GLfloat>(data_));
}
}

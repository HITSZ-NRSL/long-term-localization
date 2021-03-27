#include <cassert>
#include <vector>

#include "GlProgram.h"
#include "GlTransformFeedback.h"

namespace glow {

GLuint GlProgram::boundProgram_ = 0;

GlProgram::GlProgram() {
  id_ = glCreateProgram();
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteProgram(*ptr);
    delete ptr;
  });
}

void GlProgram::attach(const GlShader& shader) {
  if (shaders_.find(shader.type_) != shaders_.end()) shaders_.erase(shader.type_);
  shaders_.emplace(shader.type_, shader);
}

void GlProgram::attach(const GlTransformFeedback& feedback) {
  feedbacks_.push_back(feedback);
}

void GlProgram::link() {
  if (shaders_.find(ShaderType::VERTEX_SHADER) == shaders_.end() ||
      shaders_.find(ShaderType::FRAGMENT_SHADER) == shaders_.end())
    throw GlProgramError("Program must have attached at least VERTEX_SHADER and FRAGMENT_SHADER");

  for (auto it = shaders_.cbegin(); it != shaders_.cend(); ++it) {
    glAttachShader(id_, it->second.id_);
    CheckGlError();
  }

  // produce warnings for unmatched inputs or outputs.
  std::vector<ShaderType> ordering{ShaderType::VERTEX_SHADER, ShaderType::GEOMETRY_SHADER, ShaderType::FRAGMENT_SHADER};
  std::vector<std::string> names{"vertex shader", "geometry shader", "fragment shader"};
  for (uint32_t i = 0; i < ordering.size(); ++i) {
    if (shaders_.find(ordering[i]) == shaders_.end()) continue;
    const GlShader& shader = shaders_.find(ordering[i])->second;
    if (shader.inAttribs_.size() == 0) continue;  // nothing to match.

    for (auto attrib : shader.inAttribs_) {
      bool unmatched = true;
      for (int32_t j = i - 1; j >= 0; --j) {
        if (shaders_.find(ordering[j]) == shaders_.end()) continue;
        const GlShader& prevShader = shaders_.find(ordering[j])->second;
        for (auto prevAttrib : prevShader.outAttribs_) {
          if (prevAttrib.type == attrib.type && prevAttrib.name == prevAttrib.name) unmatched = false;
        }
        if (!unmatched) break;
      }

      if (unmatched) {
        std::cerr << "Warning: Unmatched attribute " << attrib.name << " in " << names[i] << "(" << shader.filename
                  << ") found" << std::endl;
      }
    }
  }

  // register varyings.
  for (auto feedback : feedbacks_) feedback.registerVaryings(id_);

  glLinkProgram(id_);

  // check if linking was successful.
  GLint isLinked = 0;
  glGetProgramiv(id_, GL_LINK_STATUS, static_cast<GLint*>(&isLinked));
  if (isLinked == GL_FALSE) {
    GLint length = 0;
    glGetProgramiv(id_, GL_INFO_LOG_LENGTH, &length);

    // The maxLength includes the NULL character
    std::vector<GLchar> error_string(length);
    glGetProgramInfoLog(id_, length, &length, &error_string[0]);

    for (auto it = shaders_.cbegin(); it != shaders_.cend(); ++it) glDetachShader(id_, it->second.id_);

    throw GlProgramError(std::string(&error_string[0], length));
  }

  linked_ = true;

  // cleanup: detach + remove shader from map.
  for (auto it = shaders_.cbegin(); it != shaders_.cend(); ++it) glDetachShader(id_, it->second.id_);
  shaders_.clear();

  // inform program that feedbacks that linking was successful.
  for (auto feedback : feedbacks_) *(feedback.linked_) = true;

  feedbacks_.clear();  // not needed anymore.
}

void GlProgram::bind() {
  assert(linked_ && "GlProgram should be linked with link() before usage!");
  glUseProgram(id_);
  boundProgram_ = id_;
}

void GlProgram::release() {
  glUseProgram(0);
  boundProgram_ = 0;
}

void GlProgram::setUniform(const GlAbstractUniform& uniform) {
  if (!linked_) throw GlProgramError("Unable to set Uniform: program not linked!");
  GLuint id = bindTransparently();
  uniform.bind(id_);
  releaseTransparently(id);
}

GLuint GlProgram::bindTransparently() {
  assert(linked_ && "GlProgram should be linked with link() before usage!");
  if (boundProgram_ == id_) return id_;

  glUseProgram(id_);
  return boundProgram_;
}

void GlProgram::releaseTransparently(GLuint oldProgram) {
  if (oldProgram == id_) return;

  glUseProgram(oldProgram);
}

} /* namespace rv */

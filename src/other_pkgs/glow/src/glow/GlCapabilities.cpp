#include "GlCapabilities.h"

#include <sstream>

namespace glow {

std::unique_ptr<GlCapabilities> GlCapabilities::instance_ = nullptr;

template <>
void GlCapabilities::initGlParameter<int>(GLenum name, uint32_t num) {
  int32_t v[4];
  glGetIntegerv(name, v);
  if (num == 1) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0])));
  if (num == 2) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1])));
  if (num == 3) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2])));
  if (num == 4) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2], v[3])));
}

template <>
void GlCapabilities::initGlParameter<float>(GLenum name, uint32_t num) {
  float v[4];
  glGetFloatv(name, v);
  if (num == 1) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0])));
  if (num == 2) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1])));
  if (num == 3) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2])));
  if (num == 4) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2], v[3])));
}

template <>
void GlCapabilities::initGlParameter<bool>(GLenum name, uint32_t num) {
  GLboolean v[4];
  glGetBooleanv(name, v);
  if (num == 1) state_.insert(std::make_pair(name, GlState::GlStateVariable(static_cast<bool>(v[0]))));
}

GlCapabilities::GlCapabilities() {
  // query capabilites.

  // Framebuffers.

  initGlParameter<bool>(GL_DOUBLEBUFFER, 1);
  initGlParameter<int>(GL_MAX_COLOR_ATTACHMENTS, 1);
  initGlParameter<int>(GL_MAX_COLOR_TEXTURE_SAMPLES, 1);
  initGlParameter<int>(GL_MAX_DEPTH_TEXTURE_SAMPLES, 1);
  initGlParameter<int>(GL_MAX_DRAW_BUFFERS, 1);
  // GL_MAX_DUAL_SOURCE_DRAW_BUFFERS
  // OpenGL 4.3: GL_MAX_FRAMEBUFFER_HEIGHT
  // OpenGL 4.3: GL_MAX_FRAMEBUFFER_LAYERS
  // OpenGL 4.3: GL_MAX_FRAMEBUFFER_SAMPLES
  // OpenGL 4.3: GL_MAX_FRAMEBUFFER_WIDTH
  // GL_MAX_INTEGER_SAMPLES
  // GL_MAX_SAMPLES

  // Programs:
  initGlParameter<int>(GL_MAX_ATOMIC_COUNTER_BUFFER_SIZE, 1);
  initGlParameter<int>(GL_MAX_COMBINED_SHADER_OUTPUT_RESOURCES, 1);
  initGlParameter<int>(GL_MAX_COMBINED_SHADER_STORAGE_BLOCKS, 1);
  initGlParameter<int>(GL_MAX_IMAGE_SAMPLES, 1);
  initGlParameter<int>(GL_MAX_IMAGE_UNITS, 1);
  initGlParameter<int>(GL_MAX_PROGRAM_TEXEL_OFFSET, 1);
  //  initGlParameter<int>(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, 1); INT64!
  initGlParameter<int>(GL_MAX_SUBROUTINES, 1);
  initGlParameter<int>(GL_MAX_SUBROUTINE_UNIFORM_LOCATIONS, 1);
  initGlParameter<int>(GL_MAX_UNIFORM_BLOCK_SIZE, 1);
  initGlParameter<int>(GL_MAX_UNIFORM_LOCATIONS, 1);
  initGlParameter<int>(GL_MAX_VARYING_VECTORS, 1);
  initGlParameter<int>(GL_MAX_VERTEX_ATTRIB_RELATIVE_OFFSET, 1);
  initGlParameter<int>(GL_MAX_VERTEX_ATTRIB_BINDINGS, 1);
  initGlParameter<int>(GL_MAX_VERTEX_ATTRIB_STRIDE, 1);
  initGlParameter<int>(GL_MIN_PROGRAM_TEXEL_OFFSET, 1);
  initGlParameter<int>(GL_NUM_PROGRAM_BINARY_FORMATS, 1);
  initGlParameter<int>(GL_NUM_SHADER_BINARY_FORMATS, 1);
  // Rasterization:
  initGlParameter<float>(GL_ALIASED_LINE_WIDTH_RANGE, 2);
  initGlParameter<float>(GL_POINT_SIZE_RANGE, 2);
  initGlParameter<float>(GL_SMOOTH_LINE_WIDTH_RANGE, 2);

  // TODO: shader execution, Comptue, Fragment, Geometry, Tesselation, Vertex, etc.

  // Texture:

  // TODO: GL_COMPRESSED_TEXTURE_FORMATS
  initGlParameter<int>(GL_MAX_TEXTURE_IMAGE_UNITS, 1);
  initGlParameter<int>(GL_MAX_3D_TEXTURE_SIZE, 1);
  initGlParameter<int>(GL_MAX_ARRAY_TEXTURE_LAYERS, 1);
  initGlParameter<int>(GL_MAX_CUBE_MAP_TEXTURE_SIZE, 1);
  initGlParameter<int>(GL_MAX_RECTANGLE_TEXTURE_SIZE, 1);
  initGlParameter<int>(GL_MAX_TEXTURE_BUFFER_SIZE, 1);
  initGlParameter<float>(GL_MAX_TEXTURE_LOD_BIAS, 1);
  initGlParameter<int>(GL_MAX_TEXTURE_SIZE, 1);
  initGlParameter<int>(GL_NUM_COMPRESSED_TEXTURE_FORMATS, 1);
  initGlParameter<int>(GL_TEXTURE_BUFFER_OFFSET_ALIGNMENT, 1);


  // Transformation state.
  initGlParameter<int>(GL_MAX_CLIP_DISTANCES, 1);
  initGlParameter<float>(GL_MAX_VIEWPORT_DIMS, 2);
  // OpenGl 4.1: initGlParameter<int>(GL_MAX_VIEWPORTS, 1);
  // OpenGl 4.1: initGlParameter<float>(GL_VIEWPORT_BOUNDS_RANGE, 2);
  // OpenGl 4.1: initGlParameter<int>(GL_VIEWPORT_SUBPIXEL_BITS, 1);

  // Vertex arrays:
  // OpenGL 4.3: initGlParameter<int>(GL_MAX_ELEMENT_INDEX, 1);
  initGlParameter<int>(GL_MAX_ELEMENTS_INDICES, 1);
  initGlParameter<int>(GL_MAX_ELEMENTS_VERTICES, 1);
  // initGlParameter<bool>(GL_PRIMITIVE_RESTART_FOR_PATCHES_SUPPORTED, 1); ???

  // TODO: > OpenGL 4.0? GL_PROGRAM_BINARY_FORMATS, GL_SHADER_BINARY_FORMATS, GL_SHADER_COMPILER,
  // ??? GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT+

  initGlParameter<int>(GL_MAX_TRANSFORM_FEEDBACK_BUFFERS, 1);
}

GlCapabilities& GlCapabilities::getInstance() {
  if (instance_ == nullptr) {
    instance_ = std::unique_ptr<GlCapabilities>(new GlCapabilities());
  }

  return *instance_;
}

std::ostream& operator<<(std::ostream& out, const GlCapabilities& cap) {
  for (auto it = cap.state_.begin(); it != cap.state_.end(); ++it) {
    out.width(40);
    out << cap.stringify_name(it->first);
    out.width(0);
    out << ": " << cap.stringify_value(*it) << std::endl;
  }

  return out;
}

#undef CASE
#define CASE(name) \
  case name:       \
    return #name;
std::string GlCapabilities::stringify_name(GLenum name) const {
  switch (name) {
    CASE(GL_MAX_COLOR_ATTACHMENTS)
    CASE(GL_MAX_COLOR_TEXTURE_SAMPLES)
    CASE(GL_MAX_3D_TEXTURE_SIZE)
    CASE(GL_MAX_DRAW_BUFFERS)
    CASE(GL_MAX_ATOMIC_COUNTER_BUFFER_SIZE)
    CASE(GL_MAX_COMBINED_SHADER_OUTPUT_RESOURCES)
    CASE(GL_MAX_COMBINED_SHADER_STORAGE_BLOCKS)
    CASE(GL_MAX_IMAGE_SAMPLES)
    CASE(GL_MAX_IMAGE_UNITS)
    CASE(GL_MAX_TEXTURE_SIZE)
    CASE(GL_MAX_PROGRAM_TEXEL_OFFSET)
    CASE(GL_MAX_SHADER_STORAGE_BLOCK_SIZE)
    CASE(GL_MAX_SUBROUTINES)
    CASE(GL_MAX_SUBROUTINE_UNIFORM_LOCATIONS)
    CASE(GL_MAX_UNIFORM_BLOCK_SIZE)
    CASE(GL_MAX_UNIFORM_LOCATIONS)
    CASE(GL_MAX_VARYING_VECTORS)
    CASE(GL_MAX_VERTEX_ATTRIB_RELATIVE_OFFSET)
    CASE(GL_MAX_VERTEX_ATTRIB_BINDINGS)
    CASE(GL_MAX_VERTEX_ATTRIB_STRIDE)
    CASE(GL_MIN_PROGRAM_TEXEL_OFFSET)
    CASE(GL_NUM_PROGRAM_BINARY_FORMATS)
    CASE(GL_NUM_SHADER_BINARY_FORMATS)
    CASE(GL_POINT_SIZE_RANGE)
    CASE(GL_LINE_WIDTH_RANGE)
    CASE(GL_DOUBLEBUFFER)
    CASE(GL_MAX_CLIP_PLANES)
    CASE(GL_MAX_VIEWPORT_DIMS)
    CASE(GL_MAX_ELEMENTS_VERTICES)
    CASE(GL_MAX_ELEMENTS_INDICES)
    CASE(GL_ALIASED_LINE_WIDTH_RANGE)
    CASE(GL_MAX_RECTANGLE_TEXTURE_SIZE)
    CASE(GL_MAX_TEXTURE_LOD_BIAS)
    CASE(GL_MAX_CUBE_MAP_TEXTURE_SIZE)
    CASE(GL_NUM_COMPRESSED_TEXTURE_FORMATS)
    CASE(GL_MAX_TEXTURE_IMAGE_UNITS)
    CASE(GL_MAX_ARRAY_TEXTURE_LAYERS)
    CASE(GL_MAX_TEXTURE_BUFFER_SIZE)
    CASE(GL_MAX_DEPTH_TEXTURE_SAMPLES)
    CASE(GL_TEXTURE_BUFFER_OFFSET_ALIGNMENT)
  }

  std::stringstream sstr;
  sstr << name;

  return sstr.str();
}
#undef CASE

std::string GlCapabilities::stringify_value(const std::pair<GLenum, GlState::GlStateVariable>& entry) const {
  std::stringstream sstr;

  if (entry.second.type == GlState::GlStateVariable::INT) {
    for (uint32_t i = 0; i < entry.second.size; ++i) sstr << ((i > 0) ? ", " : "") << entry.second.vali[i];
  } else if (entry.second.type == GlState::GlStateVariable::FLOAT) {
    for (uint32_t i = 0; i < entry.second.size; ++i) sstr << ((i > 0) ? ", " : "") << entry.second.valf[i];
  } else if (entry.second.type == GlState::GlStateVariable::BOOL) {
    sstr << (entry.second.valb ? "true" : "false");
  }
  return sstr.str();
}

} /* namespace rv */

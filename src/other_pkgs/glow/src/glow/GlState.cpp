#include "GlState.h"

#include <sstream>
#include <cassert>
#include <cmath>

namespace glow {

template <>
void GlState::initGlParameter<int>(GLenum name, uint32_t num) {
  int32_t v[4];
  glGetIntegerv(name, v);
  if (num == 1) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0])));
  if (num == 2) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1])));
  if (num == 3) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2])));
  if (num == 4) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2], v[3])));
}

template <>
void GlState::initGlParameter<float>(GLenum name, uint32_t num) {
  float v[4];
  glGetFloatv(name, v);
  if (num == 1) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0])));
  if (num == 2) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1])));
  if (num == 3) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2])));
  if (num == 4) state_.insert(std::make_pair(name, GlState::GlStateVariable(v[0], v[1], v[2], v[3])));
}

template <>
void GlState::initGlParameter<bool>(GLenum name, uint32_t num) {
  GLboolean v[4];
  glGetBooleanv(name, v);
  if (num == 1) state_.insert(std::make_pair(name, GlState::GlStateVariable(static_cast<bool>(v[0]))));
}

#undef CASE
#define CASE(name) \
  case name:       \
    return #name;

// TODO: use stringizing operator macro, e.g.,  #value, instead?
std::string GlState::stringify_name(GLenum value) const {
  switch (value) {
    CASE(GL_DRAW_FRAMEBUFFER_BINDING)
    CASE(GL_READ_FRAMEBUFFER_BINDING)
    CASE(GL_VERTEX_ARRAY_BINDING)
    CASE(GL_CURRENT_PROGRAM)
    CASE(GL_ARRAY_BUFFER_BINDING)
    CASE(GL_DRAW_INDIRECT_BUFFER_BINDING)
    CASE(GL_ELEMENT_ARRAY_BUFFER_BINDING)
    CASE(GL_QUERY_BUFFER_BINDING)
    CASE(GL_DEPTH_FUNC)
    CASE(GL_DEPTH_TEST)
    CASE(GL_COLOR_CLEAR_VALUE)
    CASE(GL_DEPTH_CLEAR_VALUE)
    CASE(GL_DEPTH_WRITEMASK)
    CASE(GL_DRAW_BUFFER)
    CASE(GL_RENDERBUFFER_BINDING)
    CASE(GL_BLEND)
    CASE(GL_BLEND_COLOR)
    CASE(GL_PROGRAM_PIPELINE_BINDING)
    CASE(GL_CULL_FACE)
    CASE(GL_CULL_FACE_MODE)
    CASE(GL_FRONT_FACE)
    CASE(GL_LINE_SMOOTH)
    CASE(GL_LINE_WIDTH)
    CASE(GL_POINT_FADE_THRESHOLD_SIZE)
    CASE(GL_POINT_SIZE)
    CASE(GL_POINT_SIZE_GRANULARITY)
    CASE(GL_POINT_SPRITE_COORD_ORIGIN)
    CASE(GL_POLYGON_MODE)
    CASE(GL_POLYGON_OFFSET_FACTOR)
    CASE(GL_POLYGON_OFFSET_FILL)
    CASE(GL_POLYGON_OFFSET_LINE)
    CASE(GL_POLYGON_OFFSET_POINT)
    CASE(GL_POLYGON_OFFSET_UNITS)
    CASE(GL_POLYGON_SMOOTH)
    CASE(GL_PROGRAM_POINT_SIZE)
    CASE(GL_RASTERIZER_DISCARD)
    CASE(GL_ACTIVE_TEXTURE)
    CASE(GL_SAMPLER_BINDING)
    CASE(GL_TEXTURE_BINDING_1D)
    CASE(GL_TEXTURE_BINDING_1D_ARRAY)
    CASE(GL_TEXTURE_BINDING_2D)
    CASE(GL_TEXTURE_BINDING_2D_ARRAY)
    CASE(GL_TEXTURE_BINDING_2D_MULTISAMPLE)
    CASE(GL_TEXTURE_BINDING_2D_MULTISAMPLE_ARRAY)
    CASE(GL_TEXTURE_BINDING_3D)
    CASE(GL_TEXTURE_BINDING_BUFFER)
    CASE(GL_TEXTURE_BINDING_CUBE_MAP)
    CASE(GL_TEXTURE_BINDING_RECTANGLE)
    CASE(GL_TEXTURE_CUBE_MAP_SEAMLESS)
    CASE(GL_DEPTH_CLAMP)
    CASE(GL_DEPTH_RANGE)
    CASE(GL_TRANSFORM_FEEDBACK_BINDING)
    CASE(GL_VIEWPORT)
    CASE(GL_PRIMITIVE_RESTART)
    CASE(GL_PRIMITIVE_RESTART_FIXED_INDEX)
    CASE(GL_PRIMITIVE_RESTART_INDEX)
    // capabilities?:
    CASE(GL_LINE_WIDTH_GRANULARITY)
    CASE(GL_SUBPIXEL_BITS)

    // TODO: remaining enums.
  }

  std::stringstream sstr;
  if ((value >= GL_SAMPLER_BINDING + GL_TEXTURE0) && (value < GL_SAMPLER_BINDING + GL_TEXTURE0 + 32)) {
    uint32_t idx = value - (GL_SAMPLER_BINDING + GL_TEXTURE0);
    sstr << "GL_SAMPLER_BINDING" << idx;

    return sstr.str();
  }

  sstr << value;

  return sstr.str();
}

#undef CASE

std::string GlState::stringify_value(const std::pair<GLenum, GlState::GlStateVariable>& entry) const {
  std::stringstream sstr;
  int32_t vali;

  switch (entry.first) {
    case GL_ACTIVE_TEXTURE:
      sstr << "GL_TEXTURE" << (entry.second.vali[0] - GL_TEXTURE0);
      break;
    case GL_DEPTH_FUNC:
      vali = entry.second.vali[0];

      if (vali == GL_NEVER) return "GL_NEVER";
      if (vali == GL_LESS) return "GL_LESS";
      if (vali == GL_EQUAL) return "GL_EQUAL";
      if (vali == GL_LEQUAL) return "GL_LEQUAL";
      if (vali == GL_GREATER) return "GL_GREATER";
      if (vali == GL_NOTEQUAL) return "GL_NOTEQUAL";
      if (vali == GL_GEQUAL) return "GL_GEQUAL";
      if (vali == GL_ALWAYS) return "GL_ALWAYS";

      sstr << "Undefined: " << vali;
      break;
    case GL_DRAW_BUFFER:
      vali = entry.second.vali[0];

      if (vali == GL_NONE) return "GL_NONE";
      if (vali == GL_FRONT_LEFT) return "GL_FRONT_LEFT";
      if (vali == GL_FRONT_RIGHT) return "GL_FRONT_RIGHT";
      if (vali == GL_BACK_LEFT) return "GL_BACK_LEFT";
      if (vali == GL_BACK_RIGHT) return "GL_BACK_RIGHT";
      if (vali == GL_FRONT) return "GL_FRONT";
      if (vali == GL_BACK) return "GL_BACK";
      if (vali == GL_LEFT) return " GL_LEFT";
      if (vali == GL_RIGHT) return "GL_RIGHT";
      if (vali == GL_FRONT_AND_BACK) return "GL_FRONT_AND_BACK";
      if (vali >= GL_COLOR_ATTACHMENT0 && vali <= GL_COLOR_ATTACHMENT15) {
        sstr << "GL_COLOR_ATTACHMENT" << (vali - GL_COLOR_ATTACHMENT0);
        break;
      }

      sstr << "Undefined: " << vali;
      break;
    default:
      if (entry.second.type == GlState::GlStateVariable::INT) {
        for (uint32_t i = 0; i < entry.second.size; ++i) sstr << ((i > 0) ? ", " : "") << entry.second.vali[i];
      } else if (entry.second.type == GlState::GlStateVariable::FLOAT) {
        for (uint32_t i = 0; i < entry.second.size; ++i) sstr << ((i > 0) ? ", " : "") << entry.second.valf[i];
      } else if (entry.second.type == GlState::GlStateVariable::BOOL) {
        sstr << (entry.second.valb ? "true" : "false");
      }
      break;
  }
  return sstr.str();
}

GlState::GlState() {
}

template <>
std::string GlState::get<std::string>(GLenum variable) const {
  if (state_.find(variable) == state_.end()) throw std::runtime_error("No such variable found in GlState.");
  auto it = state_.find(variable);

  return stringify_value(*it);
}

void GlState::restore() {
  assert(false && "not implemented.");
}

GlState GlState::queryAll() {
  // see also: https://www.opengl.org/wiki/GLAPI/glGet

  GlState state;
  // Buffer binding state.
  state.initGlParameter<int>(GL_ARRAY_BUFFER_BINDING, 1);
  state.initGlParameter<int>(GL_ELEMENT_ARRAY_BUFFER_BINDING, 1);
  state.initGlParameter<int>(GL_VERTEX_ARRAY_BINDING, 1);
#if __GL_VERSION >= 400L
  state.initGlParameter<int>(GL_DRAW_INDIRECT_BUFFER_BINDING, 1);
#endif
#if __GL_VERSION >= 440L
  state.initGlParameter<int>(GL_QUERY_BUFFER_BINDING, 1);
#endif

  // Framebuffers.
  state.initGlParameter<float>(GL_COLOR_CLEAR_VALUE, 4);
  state.initGlParameter<float>(GL_DEPTH_CLEAR_VALUE, 1);

  state.initGlParameter<int>(GL_DEPTH_FUNC, 1);
  state.initGlParameter<bool>(GL_DEPTH_TEST, 1);
  state.initGlParameter<bool>(GL_DEPTH_WRITEMASK, 1);

  state.initGlParameter<int>(GL_DRAW_BUFFER, 1);
  state.initGlParameter<int>(GL_DRAW_FRAMEBUFFER_BINDING, 1);
  // TODO: GL_READ_BUFFER
  state.initGlParameter<int>(GL_RENDERBUFFER_BINDING, 1);
  // TODO: GL_STENCIL ...

  // TODO: Hints, Image State, Multisampling,

  // Pixel Operations.
  state.initGlParameter<bool>(GL_BLEND, 1);
  state.initGlParameter<float>(GL_BLEND_COLOR, 4);

  // TODO: GL_BLEND_DST_ALPHA, GL_BLEND_DST_RGB, GL_BLEND_EQUATION_RGB, GL_BLEND_EQUATION_ALPHA,
  // TODO: GL_BLEND_SRC_ALPHA, GL_BLEND_SRC_RGB, GL_COLOR_LOGIC_OP, GL_DITHER, GL_LOGIC_OP_MODE,
  // TODO: GL_SCISSOR_BOX, GL_SCISSOR_TEST

  // TODO: Pixel Transfer Operations

  // Programs:
  state.initGlParameter<int>(GL_CURRENT_PROGRAM, 1);
  state.initGlParameter<int>(GL_PROGRAM_PIPELINE_BINDING, 1);

  // OpenGL 4.3: GL_PROGRAM_BINARY_FORMATS
  // OpenGL 4.1: GL_PROGRAM_PIPELINE_BINDING

  // Rasterization:
  state.initGlParameter<bool>(GL_CULL_FACE, 1);
  state.initGlParameter<int>(GL_CULL_FACE_MODE, 1);
  state.initGlParameter<int>(GL_FRONT_FACE, 1);
  state.initGlParameter<bool>(GL_LINE_SMOOTH, 1);
  state.initGlParameter<float>(GL_LINE_WIDTH, 1);
  state.initGlParameter<float>(GL_POINT_FADE_THRESHOLD_SIZE, 1);
  state.initGlParameter<float>(GL_POINT_SIZE, 1);
  state.initGlParameter<float>(GL_POINT_SIZE_GRANULARITY, 1);
  state.initGlParameter<int>(GL_POINT_SPRITE_COORD_ORIGIN, 1);
  //  state.initGlParameter<int>(GL_POLYGON_MODE, 2);
  state.initGlParameter<float>(GL_POLYGON_OFFSET_FACTOR, 1);
  state.initGlParameter<bool>(GL_POLYGON_OFFSET_FILL, 1);
  state.initGlParameter<bool>(GL_POLYGON_OFFSET_LINE, 1);
  state.initGlParameter<bool>(GL_POLYGON_OFFSET_POINT, 1);
  state.initGlParameter<float>(GL_POLYGON_OFFSET_UNITS, 1);

  state.initGlParameter<bool>(GL_POLYGON_SMOOTH, 1);
  state.initGlParameter<bool>(GL_PROGRAM_POINT_SIZE, 1);
  state.initGlParameter<bool>(GL_RASTERIZER_DISCARD, 1);
  state.initGlParameter<float>(GL_SMOOTH_LINE_WIDTH_GRANULARITY, 1);
  state.initGlParameter<int>(GL_SUBPIXEL_BITS, 1);

  // Texture:
  state.initGlParameter<int>(GL_ACTIVE_TEXTURE, 1);
  // query all the sampler bindings:
  int32_t max_texunits;
  glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &max_texunits);
  for (int32_t i = 0; i < max_texunits; ++i) {
    int32_t v;
    glActiveTexture(GL_TEXTURE0 + i);
    glGetIntegerv(GL_SAMPLER_BINDING, &v);
    state.state_.insert(std::make_pair(GL_SAMPLER_BINDING + GL_TEXTURE0 + i, GlState::GlStateVariable(v)));
  }
  glActiveTexture(state.get<int>(GL_ACTIVE_TEXTURE));
  // restore original state.
  state.initGlParameter<int>(GL_TEXTURE_BINDING_1D, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_1D_ARRAY, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_2D, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_2D_ARRAY, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_2D_MULTISAMPLE, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_2D_MULTISAMPLE_ARRAY, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_3D, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_BUFFER, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_CUBE_MAP, 1);
  state.initGlParameter<int>(GL_TEXTURE_BINDING_RECTANGLE, 1);
  state.initGlParameter<bool>(GL_TEXTURE_CUBE_MAP_SEAMLESS, 1);

  // Transformation state:
  state.initGlParameter<bool>(GL_DEPTH_CLAMP, 1);
  state.initGlParameter<float>(GL_DEPTH_RANGE, 1);
#if __GL_VERSION >= 400L
  state.initGlParameter<int>(GL_TRANSFORM_FEEDBACK_BINDING, 1);
#endif
  state.initGlParameter<int>(GL_VIEWPORT, 4);

  // Vertex arrays.
  state.initGlParameter<bool>(GL_PRIMITIVE_RESTART, 1);
#if __GL_VERSION >= 430L
  state.initGlParameter<bool>(GL_PRIMITIVE_RESTART_FIXED_INDEX, 1);
#endif
  state.initGlParameter<int>(GL_PRIMITIVE_RESTART_INDEX, 1);
  state.initGlParameter<int>(GL_VERTEX_ARRAY_BINDING, 1);

  // TODO: GL_VERTEX_BINDING_DIVISOR , GL_VERTEX_BINDING_OFFSET, GL_VERTEX_BINDING_STRIDE
  CheckGlError();

  return state;

  //  return GlState::query(GL_ACTIVE_TEXTURE, GL_CONTEXT_FLAGS, GL_CURRENT_PROGRAM, GL_DEPTH_TEST,
  //  GL_DRAW_FRAMEBUFFER_BINDING, GL_READ_FRAMEBUFFER_BINDING, GL_READ_FRAMEBUFFER_BINDING, GL_VERTEX_ARRAY_BINDING,
  //  GL_VIEWPORT);
}

GlState::GlStateVariable::GlStateVariable(bool value) : type(BOOL), valb{value}, size(1) {
}

GlState::GlStateVariable::GlStateVariable(int32_t i1) : type(INT), vali{i1, 0, 0, 0}, size(1) {
}
GlState::GlStateVariable::GlStateVariable(int32_t i1, int32_t i2) : type(INT), vali{i1, i2, 0, 0}, size(2) {
}

GlState::GlStateVariable::GlStateVariable(int32_t i1, int32_t i2, int32_t i3)
    : type(INT), vali{i1, i2, i3, 0}, size(3) {
}

GlState::GlStateVariable::GlStateVariable(int32_t i1, int32_t i2, int32_t i3, int32_t i4)
    : type(INT), vali{i1, i2, i3, i4}, size(4) {
}

GlState::GlStateVariable::GlStateVariable(float f1) : type(FLOAT), valf{f1, 0.0f, 0.0f, 0.0f}, size(1) {
}

GlState::GlStateVariable::GlStateVariable(float f1, float f2) : type(FLOAT), valf{f1, f2, 0.0f, 0.0f}, size(2) {
}

GlState::GlStateVariable::GlStateVariable(float f1, float f2, float f3) : type(FLOAT), valf{f1, f2, f3, 0.0f}, size(3) {
}

GlState::GlStateVariable::GlStateVariable(float f1, float f2, float f3, float f4)
    : type(FLOAT), valf{f1, f2, f3, f4}, size(4) {
}

bool GlState::GlStateVariable::operator==(const GlStateVariable& other) const {
  if (type != other.type) return false;
  if (size != other.size) return false;

  const float EPSILON = 0.0000001f;  // FIXME: have global epsilon value?

  if (type == DataType::BOOL) {
    return (valb == other.valb);
  } else if (type == DataType::INT) {
    for (uint32_t i = 0; i < size; ++i) {
      if (vali[i] != other.vali[i]) return false;
    }
  } else if (type == DataType::FLOAT) {
    for (uint32_t i = 0; i < size; ++i)
      if (std::abs(valf[i] - other.valf[i]) > EPSILON) return false;
  }

  return true;
}

bool GlState::GlStateVariable::operator!=(const GlStateVariable& other) const {
  return !(*this == other);
}

std::ostream& operator<<(std::ostream& stream, const GlState& state) {
  for (auto pair : state.state_) {
    stream.width(40);
    stream << state.stringify_name(pair.first);
    stream.width(0);
    stream << ": " << state.stringify_value(pair) << std::endl;
  }

  return stream;
}

void GlState::difference(const GlState& other) const {
  for (auto it = state_.begin(); it != state_.end(); ++it) {
    if (other.state_.find(it->first) == other.state_.end()) {
      std::cerr << stringify_name(it->first) << " missing." << std::endl;
    }

    auto oit = other.state_.find(it->first);

    if (it->second != oit->second) {
      std::cerr << stringify_name(it->first) << " expected: ";
      std::cerr << stringify_value(*it) << " but got: " << stringify_value(*oit) << std::endl;
    }
  }
}

bool GlState::operator==(const GlState& other) const {
  if (state_.size() != other.state_.size()) return false;

  for (auto it = state_.begin(); it != state_.end(); ++it) {
    if (other.state_.find(it->first) == other.state_.end()) return false;
    auto oit = other.state_.find(it->first);

    if (it->second != oit->second) return false;
  }

  return true;
}

bool GlState::operator!=(const GlState& other) const {
  return !(*this == other);
}

// GlState GlState::query(GLenum vars...)
//{
//  va_list args;
//  va_start(args, vars);
//
//  while()
//}
}
/* namespace rv */

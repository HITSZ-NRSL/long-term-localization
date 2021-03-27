#include "GlQuery.h"
#include "glexception.h"

namespace glow {

template <>
void GlQuery::value<int32_t>(int32_t& value) const {
  GLint params;
  glGetQueryObjectiv(id_, GL_QUERY_RESULT, &params);
  value = params;
}

template <>
void GlQuery::value<uint32_t>(uint32_t& value) const {
  GLuint params;
  glGetQueryObjectuiv(id_, GL_QUERY_RESULT, &params);
  value = params;
}

GlQuery::GlQuery(QueryTarget target) : target_(static_cast<GLenum>(target)) {
  glGenQueries(1, &id_);
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteQueries(1, ptr);
    delete ptr;
  });
}

/** conversion operators. **/
GlQuery::operator uint32_t() {
  GLuint params = 0;
  glGetQueryObjectuiv(id_, GL_QUERY_RESULT, &params);

  return params;
}

GlQuery::operator int32_t() {
  GLint params = 0;
  glGetQueryObjectiv(id_, GL_QUERY_RESULT, &params);

  return params;
}

/** \brief begin indexed query for specified index.
 *  If the specified target does not support an index by definition, this function calls
 *  simply the index 0 version.
 **/
void GlQuery::begin(uint32_t index) {
  if (started_) throw GlQueryError("Query object already active.");

// if VERSION < GL_4_0 => only glBeginQuery(target_, id_); is available.
#if __GL_VERSION >= 400L
  if (target_ == GL_TIME_ELAPSED || target_ == GL_ANY_SAMPLES_PASSED || target_ == GL_SAMPLES_PASSED) index = 0;
  glBeginQueryIndexed(target_, index, id_);
  index_ = index;
#else
  glBeginQuery(target_, id_);
  index_ = 0;
#endif

  started_ = true;
}

void GlQuery::end() {
#if __GL_VERSION >= 400L
  glEndQueryIndexed(target_, index_);
#else
  glEndQuery(target_);
#endif
  started_ = false;
}

/** \brief retrieve if query result is available. **/
bool GlQuery::ready() const {
  GLint params;
  glGetQueryObjectiv(id_, GL_QUERY_RESULT_AVAILABLE, &params);

  return (params == GL_TRUE);
}

/** \brief get the value of the query with specified type. **/

void GlQuery::bind() {
  // noop.
}

void GlQuery::release() {
  // noop.
}

} /* namespace rv */

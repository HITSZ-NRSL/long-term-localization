#ifndef INCLUDE_RV_GLQUERY_H_
#define INCLUDE_RV_GLQUERY_H_

#include "GlObject.h"

namespace glow {

enum class QueryTarget {
  SAMPLES_PASSED = GL_SAMPLES_PASSED,
  ANY_SAMPLES_PASSED = GL_ANY_SAMPLES_PASSED,
  ANY_SAMPLES_PASSED_CONSERVATIVE = GL_ANY_SAMPLES_PASSED_CONSERVATIVE,
  PRIMITIVES_GENERATED = GL_PRIMITIVES_GENERATED,
  TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN = GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN,
  TIME_ELAPSED = GL_TIME_ELAPSED
};

/** \brief representation of OpenGL's query object.
 *
 *  A query object can be used to query information about OpenGL context.
 *
 *  \author behley.
 */
class GlQuery : public GlObject {
 public:
  GlQuery(QueryTarget target);

  /** conversion operators. **/
  operator uint32_t();
  operator int32_t();

  /** \brief begin indexed query for specified index.
   *  If the specified target does not support an index by definition, this function calls
   *  simply the index 0 version.
   **/
  void begin(uint32_t index = 0);
  void end();

  /** \brief retrieve if query result is available. **/
  bool ready() const;

  /** \brief get the value of the query with specified type. **/
  template <class T>
  void value(T& value) const;

 protected:
  void bind() override;
  void release() override;

  bool started_{false};
  GLenum target_;
  GLuint index_{0};
};

template <class T>
void GlQuery::value(T& value) const {
  GLint params;
  glGetQueryObjectiv(id_, GL_QUERY_RESULT, &params);

  return T(params);
}

} /* namespace rv */

#endif /* INCLUDE_RV_GLQUERY_H_ */

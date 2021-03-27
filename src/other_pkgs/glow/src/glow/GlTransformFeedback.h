#ifndef INCLUDE_RV_GLTRANSFORMFEEDBACK_H_
#define INCLUDE_RV_GLTRANSFORMFEEDBACK_H_

#include <sstream>
#include <string>

#include "GlBuffer.h"
#include "GlCapabilities.h"
#include "GlObject.h"
#include "GlQuery.h"

namespace glow {

class GlProgram;

/** \brief primitive modes for transform feedback. **/
enum class TransformFeedbackMode { POINTS = GL_POINTS, LINES = GL_LINES, TRIANGLES = GL_TRIANGLES };

/** \brief Object for transform feedbacks.
 *
 *  Transform feedbacks can be used to capture transformed vertex data before the rasterization stage,
 *  just before clipping. This enables the program to record data and use this for later processing in
 *  the rendering pipeline.
 *
 *  \see GlProgram::attach(GLTransformFeedback)
 *
 *  \author behley
 */
class GlTransformFeedback : public GlObject {
 public:
  friend class GlProgram;

  GlTransformFeedback();

  /** \brief bind transform feedback for current drawing calls. **/
  void bind() override;

  /** \brief begin transform feedback **/
  void begin(TransformFeedbackMode mode);

  /** \brief end transform feedback and return primitives written to buffer. **/
  uint32_t end();

#if __GL_VERSION >= 400L
  /** \brief pause the capture of vertex data. Only available with OpenGL 4.0+. **/
  void pause();

  /** \brief resume previously pause transform feedback. **/
  void resume();

  /** \brief draw data from previous transform feedback call. **/
  void draw(GLenum mode) const;
#endif

  /** \brief release transform feedback. **/
  void release() override;

  /** \brief add a buffer for given varyings to the transform feedback.
   *
   *  The caller has to ensure that the buffer has enough space to capture the transform feedback.
   *
   *  Adding a single buffer with a single or multiple attributes implies GL_INTERLEAVED_ATTRIBS.
   *  Adding multiple buffers with a single attribute implies GL_SEPARATE_ATTRIBS.
   *
   *  Since Open GL 4.0, it is also possible to have multiple buffers with interleaved vertex
   *  attributes, which results in the option to add multiple buffers with each multiple attributes.
   *
   *  \throws GlTransformBufferError if buffer attachment is invalid.
   **/
  template <typename T>
  void attach(const std::vector<std::string>& varyings, GlBuffer<T>& buffer);

 protected:
  /** \brief register transform feedback varyings for given program id. **/
  void registerVaryings(GLuint program_id);

  std::shared_ptr<bool> bound_;
  std::shared_ptr<bool> linked_;
  std::vector<std::pair<std::vector<std::string>, std::shared_ptr<GLuint> > > buffers_;
  GlQuery countquery_{QueryTarget::TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN};
};

template <typename T>
void GlTransformFeedback::attach(const std::vector<std::string>& varyings, GlBuffer<T>& buffer) {
  uint32_t maxBuffers = GlCapabilities::getInstance().get<int32_t>(GL_MAX_TRANSFORM_FEEDBACK_BUFFERS);
  if (buffers_.size() + 1 > maxBuffers) {
    std::stringstream msg;
    msg << "Only " << maxBuffers << " transform feedback buffers allowed. See also GL_MAX_TRANSFORM_FEEDBACK_BUFFERS.";
    throw std::runtime_error(msg.str());
  }
  buffers_.push_back(std::make_pair(varyings, buffer.ptr_));
}

} /* namespace glow */

#endif /* INCLUDE_RV_GLTRANSFORMFEEDBACK_H_ */

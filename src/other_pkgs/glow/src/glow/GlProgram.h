#ifndef INCLUDE_RV_GLPROGRAM_H_
#define INCLUDE_RV_GLPROGRAM_H_

#include "GlObject.h"
#include "GlShader.h"

#include "GlUniform.h"
#include "GlTransformFeedback.h"

#include <map>

namespace glow {

/**
 * \brief Object for OpenGL's programs binding multiple shaders.
 *
 *  Each \a GlProgram can only have a single shader for each shader stage.
 *  The method \a attach keeps track of the shaders for each stage and only the
 *  last shader for a shader stage is used for linking the program.
 *
 *  For linking of a program, all shaders are first attached, the program is
 *  linked by matching of input/output variables, and finally all shaders
 *  are detached. The detaching enables to finally delete the shaders. Unmatched
 *  input output variables or attributes of shaders will issue warnings.
 *
 *  \a GlProgram offers bind() to enable the program for subsequent usage and
 *  release() to unbind the program.
 *
 * \author behley
 */
class GlProgram : public GlObject {
 public:
  /** \brief Create empty program object. **/
  GlProgram();

  /** \brief attach a \p shader to the program.
   *
   *  Each shader stage can only have a single associated \a shader, i.e.,
   *  if more than one shader is added for a shader stage, only the last
   *  added shader is kept.
   **/
  void attach(const GlShader& shader);

  /** \brief attach \a transform feedback to the program. **/
  void attach(const GlTransformFeedback& feedback);

  /** \brief link the attached shaders.
   *
   *  Verify that at least a vertex and fragment shader is attached.
   *  Tries to link the attached shaders and if linking succeeds the
   *  shaders are detached.
   *
   *  The link process might issue warnings for unmatched input/output
   *  attributes between shaders. This should enable to avoid mistakes.
   *
   *  \throw GlProgramError, if linking fails or if mandatory shader is missing.
   **/
  void link();

  /** \brief use program for rendering. **/
  void bind() override;

  /** \brief "unuse" program **/
  void release() override;

  /** \brief set uniform value
   *
   *  \throw ProgramError, if setting of value failed, i.e., the uniform does not
   *  exist or the program is currently not linked.
   **/
  void setUniform(const GlAbstractUniform& uniform);

 protected:
  static GLuint boundProgram_;
  bool linked_{false};

  // rational: each shader stage has a single shader; replace it if not needed.
  std::map<ShaderType, GlShader> shaders_;
  std::vector<GlTransformFeedback> feedbacks_;

  GLuint bindTransparently();
  void releaseTransparently(GLuint oldProgram);
};

} /* namespace rv */

#endif /* INCLUDE_RV_GLPROGRAM_H_ */

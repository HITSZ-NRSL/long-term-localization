#ifndef INCLUDE_RV_GLEXCEPTION_H_
#define INCLUDE_RV_GLEXCEPTION_H_

/** Some custom exceptions ... **/

#include <stdexcept>

namespace glow {

/** \brief Error in shader creation or compilation. **/
class GlShaderError : public std::runtime_error {
 public:
  GlShaderError(const std::string& msg);
};

/** \brief Error in the GlProgram, most likely a linking error. */
class GlProgramError : public std::runtime_error {
 public:
  GlProgramError(const std::string& msg);
};

/** \brief Error with GlOffscreenContext: creation of object, etc. */
class GlOffscreenContextError : public std::runtime_error {
 public:
  GlOffscreenContextError(const std::string& msg);
};

/** \brief Error while using GlVertexArray: missing bind()/release(). **/
class GlVertexArrayError : public std::runtime_error {
 public:
  GlVertexArrayError(const std::string& msg);
};

/** \brief Error while using GlTexture. **/
class GlTextureError : public std::runtime_error {
 public:
  GlTextureError(const std::string& msg);
};

class GlFramebufferError : public std::runtime_error {
 public:
  GlFramebufferError(const std::string& msg);
};

class GlTransformFeedbackError : public std::runtime_error {
 public:
  GlTransformFeedbackError(const std::string& msg);
};

class GlQueryError : public std::runtime_error {
 public:
  GlQueryError(const std::string& msg);
};

class GlTextureRectangleError : public std::runtime_error {
 public:
  GlTextureRectangleError(const std::string& msg);
};
}
// ...

#endif /* INCLUDE_RV_GLEXCEPTION_H_ */

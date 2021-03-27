#include "glexception.h"

namespace glow {

GlShaderError::GlShaderError(const std::string& msg) : std::runtime_error(msg) {
}

GlProgramError::GlProgramError(const std::string& msg) : std::runtime_error(msg) {
}

GlOffscreenContextError::GlOffscreenContextError(const std::string& msg) : std::runtime_error(msg) {
}

GlVertexArrayError::GlVertexArrayError(const std::string& msg) : std::runtime_error(msg) {
}

GlTextureError::GlTextureError(const std::string& msg) : std::runtime_error(msg) {
}

GlFramebufferError::GlFramebufferError(const std::string& msg) : std::runtime_error(msg) {
}

GlTransformFeedbackError::GlTransformFeedbackError(const std::string& msg) : std::runtime_error(msg) {
}

GlQueryError::GlQueryError(const std::string& msg) : std::runtime_error(msg) {
}

GlTextureRectangleError::GlTextureRectangleError(const std::string& msg) : std::runtime_error(msg) {
}
}

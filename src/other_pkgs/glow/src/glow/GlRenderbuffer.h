#ifndef INCLUDE_RV_GLRENDERBUFFER_H_
#define INCLUDE_RV_GLRENDERBUFFER_H_

#include "GlObject.h"

namespace glow {

// forward declarations.
class GlFramebuffer;

enum class RenderbufferFormat {
  RGBA = GL_RGBA,
  RGB = GL_RGB,
  RG = GL_RG,
  R = GL_R,
  DEPTH = GL_DEPTH,
  DEPTH_STENCIL = GL_DEPTH24_STENCIL8
};

/**
 * \brief Representation of OpenGL's renderbuffer object.
 *
 *
 * Renderbuffer objects are write-only buffers for rendering purposes. You may use these as attachments
 * for frambuffers if you do not care about the rendering results. If you are interested in the results,
 * you should use GlTexture instead.
 *
 * \see GlFramebuffer
 *
 * \author behley
 */
class GlRenderbuffer : public GlObject {
 public:
  friend class GlFramebuffer;

  GlRenderbuffer(uint32_t width, uint32_t height, RenderbufferFormat fmt = RenderbufferFormat::RGB);

  void bind() override;
  void release() override;

  uint32_t width() const;
  uint32_t height() const;

 protected:
  GLenum format_;
  uint32_t width_, height_;
};

} /* namespace rv */

#endif /* INCLUDE_RV_GLRENDERBUFFER_H_ */

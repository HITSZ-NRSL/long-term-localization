#ifndef INCLUDE_RV_GLFRAMEBUFFER_H_
#define INCLUDE_RV_GLFRAMEBUFFER_H_

#include <map>
#include "GlTexture.h"
#include "GlTextureRectangle.h"
#include "GlRenderbuffer.h"

namespace glow {

enum class FramebufferTarget { BOTH = GL_FRAMEBUFFER, READ = GL_READ_FRAMEBUFFER, DRAW = GL_DRAW_FRAMEBUFFER };

enum class FramebufferAttachment {
  COLOR0 = GL_COLOR_ATTACHMENT0,
  COLOR1 = GL_COLOR_ATTACHMENT1,
  COLOR2 = GL_COLOR_ATTACHMENT2,
  COLOR3 = GL_COLOR_ATTACHMENT3,
  COLOR4 = GL_COLOR_ATTACHMENT4,
  COLOR5 = GL_COLOR_ATTACHMENT5,
  COLOR6 = GL_COLOR_ATTACHMENT6,
  // TODO: remaining color attachments,
  DEPTH = GL_DEPTH_ATTACHMENT,
  STENCIL = GL_STENCIL_ATTACHMENT,
  DEPTH_STENCIL = GL_DEPTH_STENCIL_ATTACHMENT
};

/** \brief Representation of an OpenGL's framebuffer object
 *
 *  Framebuffer objects can be used as target for (offscreen) rendering instead of the default framebuffer.
 *  However, before the framebuffer can be used as target for rendering there should be buffers added to
 *  the framebuffer. At least, COLOR0 and DEPTH + STENCIL or DEPTH_STENCIL attachments either by attaching
 *  a GlRenderbuffer or a Texture must be made. You can check with the valid method if everything needed is
 *  present.
 *
 *  TODO: integrate glDrawBuffers: attach automatically enables the buffers?
 *  different mapping or deactivating buffers must be set explicitly? map(), enable()?
 *
 *  \author behley
 */
class GlFramebuffer : public GlObject {
 public:
  /** \brief initialize an empty framebuffer object with given size (width x height) without any attachments. **/
  GlFramebuffer(uint32_t width, uint32_t height, FramebufferTarget target = FramebufferTarget::BOTH);

  ~GlFramebuffer();

  /** \brief bind framebuffer to current context.
   *
   * Bind framebuffer to current context. Each subsequent drawing operations will use the
   * framebuffer. If the framebuffer is invalid, i.e., some attachments are missing, a
   * GlFramebufferException is thrown.
   *
   * \throws GlFramebufferException invalid framebuffer object.
   *
   * \see GlFramebuffer::valid()
   */
  void bind() override;

  void release() override;

  /** \brief attach texture to given target.
   *
   *  The texture must have at least the width and height of the framebuffer object.
   **/
  void attach(FramebufferAttachment target, GlTexture& texture);

  /** \brief attach rectangular texture to given target.
   *
   *  The texture must have at least the width and height of the framebuffer object.
   **/
  void attach(FramebufferAttachment target, GlTextureRectangle& texture);

  /** \brief attach renderbuffer object to given target.
   *
   * The renderbuffer must have at least the width and height of the framebuffer object.
   */
  void attach(FramebufferAttachment target, GlRenderbuffer& buffer);

  /** \brief is Framebuffer object valid?
   *
   * Check if all needed targets are attached.
   * \return false if something is missing; true, otherwise.
   */
  bool valid() const;

  /** \brief get framebuffer width **/
  uint32_t width() const;

  /** \brief get framebuffer height **/
  uint32_t height() const;

  /** \brief resize the framebuffer and invalidate. **/
  void resize(uint32_t width, uint32_t height);

 protected:
  GLuint bindTransparently();
  void releaseTransparently(GLuint old_id);

  static GLuint boundFramebuffer_;

  GLenum target_;
  bool valid_{false};
  uint32_t width_, height_;
  // hold pointers to ensure that resources are not deleted.
  std::map<FramebufferAttachment, std::shared_ptr<GLuint> > attachments_;
};

} /* namespace rv */

#endif /* INCLUDE_RV_GLFRAMEBUFFER_H_ */

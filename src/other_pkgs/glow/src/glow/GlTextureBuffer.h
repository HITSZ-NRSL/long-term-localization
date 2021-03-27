#ifndef INCLUDE_RV_GLTEXTUREBUFFER_H_
#define INCLUDE_RV_GLTEXTUREBUFFER_H_

#include "GlBuffer.h"
#include "GlTextureFormat.h"

namespace glow {

/** \brief Texture based on a GlBuffer object.
 *
 *  A texture object, where the texture's data is stored in a buffer object.
 *
 *
 *  \author behley
 **/
class GlTextureBuffer : public GlObject {
 public:
  template <typename T>
  GlTextureBuffer(GlBuffer<T>& buffer, TextureFormat format)
      : format_(format) {
    buffer_ = buffer.ptr_;  // hold pointer to avoid deallocation before texture object is deallocated.
    glGenTextures(1, &id_);

    ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
      glDeleteTextures(1, ptr);
      delete ptr;
    });

    GLuint old_id = bindTransparently();
    GLint texFormat = static_cast<GLint>(format_);
    buffer.bind();
    glTexBuffer(GL_TEXTURE_BUFFER, texFormat, buffer.id());
    buffer.release();
    releaseTransparently(old_id);

    CheckGlError();
  }

  void bind() override {
    glBindTexture(GL_TEXTURE_BUFFER, id_);
    boundTexture_ = id_;
  }

  void release() override {
    glBindTexture(GL_TEXTURE_BUFFER, 0);
    boundTexture_ = 0;
  }

 protected:
  GLuint bindTransparently() const {
    if (boundTexture_ == id_) return id_;

    glBindTexture(GL_TEXTURE_BUFFER, id_);

    return boundTexture_;
  }

  void releaseTransparently(GLuint old_id) const {
    if (old_id == id_) return;

    glBindTexture(GL_TEXTURE_BUFFER, old_id);
  }

  static GLuint boundTexture_;
  std::shared_ptr<GLuint> buffer_;
  TextureFormat format_;
};

} /* namespace rv */

#endif /* INCLUDE_RV_GLTEXTUREBUFFER_H_ */

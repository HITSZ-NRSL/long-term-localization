#ifndef SRC_OPENGL_GLTEXTURE_H_
#define SRC_OPENGL_GLTEXTURE_H_

#include <vector>

#include "GlObject.h"
#include "GlPixelFormat.h"
#include "GlTextureFormat.h"

namespace glow {

class GlFramebuffer;
class GlTextureRectangle;

enum class TexMinOp {
  LINEAR = GL_LINEAR,
  NEAREST = GL_NEAREST,
  NEAREST_MIPMAP_NEAREST = GL_NEAREST_MIPMAP_NEAREST,
  NEAREST_MIPMAP_LINEAR = GL_NEAREST_MIPMAP_LINEAR,
  LINEAR_MIPMAP_NEAREST = GL_LINEAR_MIPMAP_NEAREST,
  LINEAR_MIPMAP_LINEAR = GL_LINEAR_MIPMAP_LINEAR
};

enum class TexMagOp { LINEAR = GL_LINEAR, NEAREST = GL_NEAREST };

enum class TexWrapOp {
  CLAMP_TO_EDGE = GL_CLAMP_TO_EDGE,
  CLAMP_TO_BORDER = GL_CLAMP_TO_BORDER,
  MIRRORED_REPEAT = GL_MIRRORED_REPEAT,
  REPEAT = GL_REPEAT,
  MIRROR_CLAMP_TO_EDGE = GL_MIRROR_CLAMP_TO_EDGE
};

enum class TexSwizzle { R = GL_RED, G = GL_GREEN, B = GL_BLUE, A = GL_ALPHA, ZERO = GL_ZERO, ONE = GL_ONE };

/**
 * \brief Wrapper for OpenGl's texture
 *
 * The class provides simpler access to texture by encapsulating most of the
 * functionality associated with a texture.
 *
 * OpenGL 4.5 Core specification:
 * > Texture objects or textures include a collection of texture images built from arrays
 * > of image elements. The image elements are referred to as texels. There are many
 * > types of texture objects varying by dimensionality and structure; [...]
 * > exture objects also include state describing the image parameters of the tex-
 * > ture images, and state describing how sampling is performed when a shader ac-
 * > cesses a texture.
 * > Shaders may sample a texture at a location indicated by specified texture co-
 * > ordinates, with details of sampling determined by the sampler state of the texture.
 * > The resulting texture samples are typically used to modify a fragment’s color, in
 * > order to map an image onto a geometric primitive being drawn, but may be used
 * > for any purpose in a shader.
 *
 * TODO: To ensure consistency: the assign, resize on non-unique objects should generate a new
 * texture object, i.e., if ptr_->unique is true, then these functions simply resue the texture object?
 *
 * TODO: Implement for each dimension a own class: GlTexture1D, GlTexture2D, GlTexture3D...this, would
 * firstly enable a more consistent enforcement of constraints for framebuffers, etc. and secondly make
 * it unnecessary to check always the dimensions of the underlying opengl texture.
 *
 * \see GlFramebuffer
 *
 * \author behley
 *
 */
class GlTexture : public GlObject {
 public:
  friend class GlFramebuffer;
  friend class GlTextureRectangle;

  /** \brief create a one-dimensional empty texture with specified internal format **/
  GlTexture(uint32_t width, TextureFormat format = TextureFormat::RGB);
  /** \brief create a two-dimensional empty texture with specified internal format **/
  GlTexture(uint32_t width, uint32_t height, TextureFormat format = TextureFormat::RGB);
  /** \brief create a three-dimensional empty texture with specified internal format **/
  GlTexture(uint32_t width, uint32_t height, uint32_t depth, TextureFormat format = TextureFormat::RGB);

  ~GlTexture();

  /** \brief generate a copy of the texture. **/
  GlTexture clone() const;

  /** \brief copy data from other to this texture.
   *
   *  If textures have different sizes, the values are interpolated by using NEAREST texture value.
   **/
  void copy(const GlTexture& other);

  /** \brief bind the texture to the currently active texture unit.
   *
   *  Use glActiveTexture(...) to activate a specific texture unit before calling bind.
   *
   *  FIXME: bind(id) and release(id);
   *   - id of release is needed as the same texture could be bound to multiple texture units.
   *   - glActivateTexture(id) could be done inside the texture...
   *   - this somehow breaks the nice thing that everything is an GlObject...
   *  FIXME: ScopedBinder with arguments?
   */
  void bind() override;

  void release() override;

  // TODO: expose remaining texture parameters by virtue of getter/setters.

  /** \brief set the filtering operation if texture is projected on smaller elements (squashed). **/
  void setMinifyingOperation(TexMinOp minifyingOperation);
  /** \brief set the filtering operation if texture is projected on larger elements (enlarged). **/
  void setMagnifyingOperation(TexMagOp magnifyingOperation);

  /** \brief set the wrapping function for accessing values outside the texture. **/
  void setWrapOperation(TexWrapOp wrap_s);
  /** \brief set the wrapping function for accessing values outside the texture. **/
  void setWrapOperation(TexWrapOp wrap_s, TexWrapOp wrap_t);
  /** \brief set the wrapping function for accessing values outside the texture. **/
  void setWrapOperation(TexWrapOp wrap_s, TexWrapOp wrap_t, TexWrapOp wrap_r);

  /** \brief specifies how to swizzle values for access.
   *
   *  Set the swizzling of color channels for access, i.e., a color component is replaced by
   *  the spefcified value.
   *
   *  For example, setTextureSwizzle(TexSwizzle::R, TexSwizzle::R, TexSwizzle::R, TexSwizzle::ONE)
   *  causes to result for every read in (R,R,R,1) instead of the R,G,B,A color inside the texture.
   */
  void setTextureSwizzle(TexSwizzle red, TexSwizzle green, TexSwizzle blue, TexSwizzle alpha);

  /** \brief Assign data to the texture. **/
  template <typename T>
  void assign(PixelFormat pixelfmt, PixelType type, T* data);

  /** \brief resizing the texture to given dimensions.
   *
   *  FIXME: resize currently causes loss of all old information.
   **/
  void resize(uint32_t width);

  /** \brief resizing the texture to given dimensions.
   *
   *  FIXME: resize currently causes loss of all old information.
   **/
  void resize(uint32_t width, uint32_t height);

  /** \brief resizing the texture to given dimensions.
   *
   *  FIXME: resize currently causes loss of all old information.
   **/
  void resize(uint32_t width, uint32_t height, uint32_t depth);

  uint32_t width() const;
  uint32_t height() const;
  uint32_t depth() const;

  /** \brief save texture to specified file with given filename.
   *
   *  Depending on the available libraries, different file types are supported. The
   *  file type is inferred from the extension of the filename.
   *
   *  \return true, if file could be written to the specified location. false, otherwise.
   **/
  bool save(const std::string& filename) const;

  /** \brief load texture from specified file with given filename.
   *
   *  Depending on the available libraries, different input file types are supported. The
   *  actual file type is inferred from the extension of the filename.
   *
   *  \throw GlTextureError
   */
  static GlTexture loadTexture(const std::string& filename);

  /** \brief download the texture to the given vector.
   *
   *  TODO: enforce correct format.
   **/
  template <typename T>
  void download(std::vector<T>& data) const;

  template <typename T>
  void download(T* ptr) const;

  template <typename T>
  void download(PixelFormat pixelfmt, T* ptr) const;

  /** \brief generate Mipmaps. **/
  void generateMipmaps();

 protected:
  //    const std::shared_ptr<GLuint>& ptr() const
  //    {
  //      return ptr_;
  //    }

  GLuint bindTransparently() const;
  void releaseTransparently(GLuint old_id) const;

  void allocateMemory();

  static uint32_t numComponents(TextureFormat format);
  static GLuint boundTexture_;

  uint32_t width_, height_{0}, depth_{0};
  GLenum target_;
  TextureFormat format_;
};

template <typename T>
void GlTexture::assign(PixelFormat pixelfmt, PixelType pixeltype, T* data) {
  GLuint old_id = bindTransparently();

  // TODO: possible to have pixeltype == T?

  if (target_ == GL_TEXTURE_1D) {
    glTexImage1D(target_, 0, static_cast<GLint>(format_), width_, 0, static_cast<GLenum>(pixelfmt),
                 static_cast<GLenum>(pixeltype), data);
  } else if (target_ == GL_TEXTURE_2D) {
    glTexImage2D(target_, 0, static_cast<GLint>(format_), width_, height_, 0, static_cast<GLenum>(pixelfmt),
                 static_cast<GLenum>(pixeltype), data);
  } else if (target_ == GL_TEXTURE_3D) {
    glTexImage3D(target_, 0, static_cast<GLint>(format_), width_, height_, depth_, 0, static_cast<GLenum>(pixelfmt),
                 static_cast<GLenum>(pixeltype), data);
  }

  releaseTransparently(old_id);
}

} /* namespace rv */

#endif /* SRC_OPENGL_GLTEXTURE_H_ */

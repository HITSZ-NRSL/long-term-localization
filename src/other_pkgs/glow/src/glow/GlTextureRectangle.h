#ifndef INCLUDE_RV_GLTEXTURERECTANGLE_H_
#define INCLUDE_RV_GLTEXTURERECTANGLE_H_

#include <vector>
#include "GlObject.h"
#include "GlPixelFormat.h"
#include "GlTextureFormat.h"

namespace glow {

// forward declaration
class GlTexture;

enum class TexRectMinOp { LINEAR = GL_LINEAR, NEAREST = GL_NEAREST };

enum class TexRectMagOp { LINEAR = GL_LINEAR, NEAREST = GL_NEAREST };

/**
-  GL_CLAMP_TO_EDGE causes s coordinates to be clamped to the range
   [1/2N,1−1/2N], where N is the size of the texture in the direction of
   clamping.
-  GL_CLAMP_TO_BORDER evaluates s coordinates in a similar manner to
   GL_CLAMP_TO_EDGE. However, in cases where clamping would have occurred in
-  GL_CLAMP_TO_EDGE mode, the fetched texel data is substituted with the
   values specified by GL_TEXTURE_BORDER_COLOR.
-  GL_REPEAT causes the integer part of the s coordinate to be ignored; the
    GL uses only the fractional part, thereby creating a repeating pattern.
-  GL_MIRRORED_REPEAT causes the s coordinate to be set to the fractional
   part of the texture coordinate if the integer part of s is even; if the
   integer part of s is odd, then the s texture coordinate is set to 1−frac(s),
   where frac(s) represents the fractional part of s.
-  GL_MIRROR_CLAMP_TO_EDGE causes the the s coordinate to be repeated as for
   GL_MIRRORED_REPEAT for one repitition of the texture, at which point the
   coordinate to be clamped as in GL_CLAMP_TO_EDGE.

**/
enum class TexRectWrapOp {
  CLAMP_TO_EDGE = GL_CLAMP_TO_EDGE,
  CLAMP_TO_BORDER = GL_CLAMP_TO_BORDER,
  MIRRORED_REPEAT = GL_MIRRORED_REPEAT,
  //  REPEAT = GL_REPEAT, -- not supported by rectangular textures.
  MIRROR_CLAMP_TO_EDGE = GL_MIRROR_CLAMP_TO_EDGE
};

enum class TexRectSwizzle { R = GL_RED, G = GL_GREEN, B = GL_BLUE, A = GL_ALPHA, ZERO = GL_ZERO, ONE = GL_ONE };

/** \brief a rectangular texture object.
 *
 *  A GlTextureRectangle is a wrapper for a texture of type GL_TEXTURE_RECTANGLE. As a
 *  rectangular texture does not do the conversion to "texture coordinates" in range
 *  [0,1], this type of texture does only support a single layer of textures and thus no
 *  mipmapping. Furthermore, only two-dimensional rectangular textures are supported by
 *  OpenGL. Apart from that, it should be the same as a "normal" texture.
 *
 *  TODO: implement compatible texture's functionality.
 *
 *  \see GlFramebuffer
 *
 *  \author behley
 **/
class GlTextureRectangle : public GlObject {
 public:
  friend class GlFramebuffer;

  /** \brief create and allocate rectangular texture of given size and format. **/
  GlTextureRectangle(uint32_t width, uint32_t height, TextureFormat);

  ~GlTextureRectangle();

  /** \brief generate a copy of the texture. **/
  GlTextureRectangle clone() const;

  /** \brief copy data from other to this texture.
   *
   *  If textures have different sizes, the values are interpolated by using NEAREST texture value.
   **/
  void copy(const GlTextureRectangle& other);

  /** \brief copy data from another texture. **/
  void copy(const GlTexture& other);

  /** \brief bind the texture to the currently active texture unit.
   *
   *  Use glActiveTexture(...) to activate a specific texture unit before calling bind.
   *
   *  FIXME: bind(id) and release(id)
   *  FIXME: ScopedBinder with arguments?
   */
  void bind() override;

  void release() override;

  // TODO: expose remaining texture parameters by virtue of getter/setters.

  /** \brief set the filtering operation if texture is projected on smaller elements (squashed). **/
  void setMinifyingOperation(TexRectMinOp minifyingOperation);
  /** \brief set the filtering operation if texture is projected on larger elements (enlarged). **/
  void setMagnifyingOperation(TexRectMagOp magnifyingOperation);

  /** \brief set the wrapping function for accessing values outside the texture. **/
  void setWrapOperation(TexRectWrapOp wrap_s, TexRectWrapOp wrap_t);

  /** \brief specifies how to swizzle values for access.
   *
   *  Set the swizzling of color channels for access, i.e., a color component is replaced by
   *  the specified value.
   *
   *  For example, setTextureSwizzle(TexSwizzle::R, TexSwizzle::R, TexSwizzle::R, TexSwizzle::ONE)
   *  causes to result for every read in (R,R,R,1) instead of the R,G,B,A color inside the texture.
   */
  void setTextureSwizzle(TexRectSwizzle red, TexRectSwizzle green, TexRectSwizzle blue, TexRectSwizzle alpha);

  /** \brief Assign data to the texture. **/
  template <typename T>
  void assign(PixelFormat pixelfmt, PixelType type, T* data);

  /** \brief resize texture to given dimensions.
   *
   * FIXME: currently resize causes loss of all data.
   */
  void resize(uint32_t width, uint32_t height);

  uint32_t width() const;
  uint32_t height() const;

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
   *  \throw GlTextureRectangle
   */
  static GlTextureRectangle loadTexture(const std::string& filename);

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

 protected:
  GLuint bindTransparently() const;
  void releaseTransparently(GLuint old_id) const;

  void allocateMemory();

  static uint32_t numComponents(TextureFormat format);
  static GLuint boundTexture_;

  uint32_t width_, height_;
  TextureFormat format_;
};

template <typename T>
void GlTextureRectangle::assign(PixelFormat pixelfmt, PixelType pixeltype, T* data) {
  GLuint old_id = bindTransparently();

  // TODO: possible to have pixeltype == T?

  glTexImage2D(GL_TEXTURE_RECTANGLE, 0, static_cast<GLint>(format_), width_, height_, 0, static_cast<GLenum>(pixelfmt),
               static_cast<GLenum>(pixeltype), data);

  releaseTransparently(old_id);
}
}
/* namespace rv */

#endif /* INCLUDE_RV_GLTEXTURERECTANGLE_H_ */

#ifndef INCLUDE_RV_GLTEXTUREFORMAT_H_
#define INCLUDE_RV_GLTEXTUREFORMAT_H_

#include "GlObject.h"

namespace glow {

/** \brief texture format definitions.
 *
 *  TODO: Define only GPU-capability compatible formats depending on the architecture.
 *
 *  See chapter 8.5.1 for required and optional texture formats. (This differs at least
 *  for OpenGL 4.3 and OpenGL 4.5...)
 *
 *  Caveat: In OpenGL 4.3 there are only RGBA*, R*, and RG* color formats need to be
 *  supported. As of OpenGL 4.5, all formats R*,RG*, RGB*, RGBA* must be supported.
 **/

enum class TextureFormat {
  // TODO: Flags and replacements for supported texture types.

  // base texture formats ("best fit decided by driver..."):
  RGBA = GL_RGBA,  // values in [0.0, 1.0]
  RGB = GL_RGB,    // values in [0.0, 1.0]
  RG = GL_RG,      // values in [0.0, 1.0]
  R = GL_RED,      // values in [0.0, 1.0]
  DEPTH = GL_DEPTH,
  DEPTH_STENCIL = GL_DEPTH24_STENCIL8,

  // Some explicit 32-bit texture formats:
  R_INTEGER = GL_R32I,
  RG_INTEGER = GL_RG32I,
  RGB_INTEGER = GL_RGB32I,
  RGBA_INTEGER = GL_RGBA32I,

  R_FLOAT = GL_R32F,
  RG_FLOAT = GL_RG32F,
  RGB_FLOAT = GL_RGB32F,
  RGBA_FLOAT = GL_RGBA32F

  // custom formats
  // TODO: INTENSITY = GL_R, // GRAY => GL_R and swizzle mask (R, R, R, 1.0)

};
}

#endif /* INCLUDE_RV_GLTEXTUREFORMAT_H_ */

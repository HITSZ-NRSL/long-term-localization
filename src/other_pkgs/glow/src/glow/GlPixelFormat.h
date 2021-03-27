#ifndef INCLUDE_RV_GLPIXELFORMAT_H_
#define INCLUDE_RV_GLPIXELFORMAT_H_

#include "GlObject.h"

/** \brief pixel format and pixel type definitions.
 *
 *
 *  \author behley
 **/

namespace glow {

enum class PixelFormat {
  // pixels colors are stored in [0.0, 1.0]
  R = GL_RED,
  RG = GL_RG,
  RGB = GL_RGB,
  BRG = GL_BGR,
  RGBA = GL_RGBA,
  BGRA = GL_BGRA,
  // pixels colors are stored in range [0,2^B-1], where B corresponds to the number of bits.
  R_INTEGER = GL_RED_INTEGER,
  RG_INTEGER = GL_RG_INTEGER,
  RGB_INTEGER = GL_RGB_INTEGER,
  BGR_INTEGER = GL_BGRA_INTEGER,
  RGBA_INTEGER = GL_RGBA_INTEGER,
  BGRA_INTEGER = GL_BGRA_INTEGER,
  DEPTH = GL_DEPTH_COMPONENT,
  DEPTH_STENCIL = GL_DEPTH24_STENCIL8,
  STENCIL = GL_STENCIL_INDEX,

};

enum class PixelType {
  UNSIGNED_BYTE = GL_UNSIGNED_BYTE,    // [0, 255]
  BYTE = GL_BYTE,                      // [-127, 128]
  UNSIGNED_SHORT = GL_UNSIGNED_SHORT,  // ...
  SHORT = GL_SHORT,
  UNSIGNED_INT = GL_UNSIGNED_INT,
  INT = GL_INT,
  HALF_FLOAT = GL_HALF_FLOAT,
  FLOAT = GL_FLOAT,                   // [0.0, 1.0]
  // TODO: add packed formats, ...
};

} /* namespace rv */

#endif

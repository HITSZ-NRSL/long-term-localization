#ifndef INCLUDE_RV_GLSAMPLER_H_
#define INCLUDE_RV_GLSAMPLER_H_

#include "GlObject.h"
#include "GlTexture.h"

namespace glow {

/** \brief Representation of OpenGL's sampler object for texture sampling options.
 *
 *  Usually, you can specify the texture sampling options right with the texture itself. However,
 *  sometimes it would be nice to have sampling parameters in one place or different sampling
 *  parameters for the same texture. This is exactly the use case of sampler objects: if a sampler
 *  object is bound to a specified texture unit,
 *
 *  TODO: have bind with multiple texture units. (glBindSamplers?)
 *  TODO: Ensure consistent setting of sampler for TEXTURE_RECTANGLE?
 *
 *  \author behley
 */
class GlSampler : public GlObject {
 public:
  GlSampler();

  /** \brief use sampler for specific texture unit identified by its index (0, 1, ...). **/
  void bind(uint32_t textureUnitId);

  /** \brief "unuse" sampler for given texture unit.  **/
  void release(uint32_t textureUnitId);

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

 protected:
  void bind() override;
  void release() override;

  uint32_t boundUnit_{0};
};
}
/* namespace rv */

#endif /* INCLUDE_RV_GLSAMPLER_H_ */

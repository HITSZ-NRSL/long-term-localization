#ifndef INCLUDE_RV_GLVERTEXARRAY_H_
#define INCLUDE_RV_GLVERTEXARRAY_H_

#include "GlObject.h"
#include "GlBuffer.h"
#include <memory>
#include <map>

namespace glow {
/** \brief data type of the vertex attribute. **/
enum class AttributeType {
  BYTE = GL_BYTE,
  UNSIGNED_BYTE = GL_UNSIGNED_BYTE,
  SHORT = GL_SHORT,
  UNSIGNED_SHORT = GL_UNSIGNED_SHORT,
  INT = GL_INT,
  UNSIGNED_INT = GL_UNSIGNED_INT,
  HALF_FLOAT = GL_HALF_FLOAT,
  FLOAT = GL_FLOAT,
  DOUBLE = GL_DOUBLE,
  FIXED = GL_FIXED,
  INT_2_10_10_10_REV = GL_INT_2_10_10_10_REV,
  UNSIGNED_INT_2_10_10_10_REV = GL_UNSIGNED_INT_2_10_10_10_REV,
  UNSIGNED_INT_10F_11F_11F_REV = GL_UNSIGNED_INT_10F_11F_11F_REV
};

/** \brief representation of a Vertex Array Object.
 *
 *  In core profile OpenGL, the vertex array object is needed for specification of the vertex
 *  attributes. Therefore, this class provides a convenient access to a Vertex Array Object
 *  with resource management.
 *
 *  It furthermore enables a more "natural" usage of vertex buffer objects with vertex arrays
 *  by making the actual definition of attributes a part of this object.
 *
 *
 *  \author behley
 *
 *  TODO: Compatibility with glBindVertexBuffer / glVertexBindingDivisor​? Available with OpenGL version 4.3
 **/
class GlVertexArray : public GlObject {
 public:
  GlVertexArray();
  ~GlVertexArray();

  /** \brief bind the vertex array object. **/
  void bind() override;
  /** \brief release the vertex array object. **/
  void release() override;

  //    template<typename T>
  //    void unbindVertexBuffer()

  /** \brief set the vertex attribute pointer for the given vertex buffer
   *
   *  Vertex attributes define the layout of the data inside memory. This actually maps
   *  an array or its contents to a attribute in the vertex shader and activates the attribute.
   *
   *  \param idx index of the vertex attribute
   *  \param buffer vertex buffer represented by a GlBuffer object.
   *  \param size size or number of components of the attribute (1,2,3, or 4)
   *  \param type data type of the attribute inside the vertex buffer.
   *  \param normalized should the data be normalized?
   *  \param stride specifies the byte offset between consecutive generic vertex attributes
   *  \param offset specifies the byte offset of the first component of the first generic vertex attribute
   *         in the array in the data store of the buffer.
   **/
  template <typename T>
  void setVertexAttribute(uint32_t idx, GlBuffer<T>& buffer, int32_t size, AttributeType type, bool normalized,
                          uint32_t stride, GLvoid* offset);

  /** \brief enable vertex attribute with given index \a idx **/
  void enableVertexAttribute(uint32_t idx);
  /** \brief disable vertex attribute with given index \a idx **/
  void disableVertexAttribute(uint32_t idx);

 protected:
  /** \brief bind vertex array object only if needed and return overwritten vertex array object. **/
  GLuint bindTransparently();
  /** \brief release vertex array object and restore state before calling bindTranparently. **/
  void releaseTransparently(GLuint old_vao);

  static GLuint boundVAO_;

  // TODO: should we also check if the vertex attributes are set when enabled?
  struct VertexAttributeState {
   public:
    GlObject* buffer{0};
    bool initialized{false};
    bool enabled{false};
  };

  // only needed for book keeping purposes.
  std::map<uint32_t, std::shared_ptr<GLuint> > vertexBuffers_;
};

template <typename T>
void GlVertexArray::setVertexAttribute(uint32_t idx, GlBuffer<T>& buffer, int32_t numComponents, AttributeType type,
                                       bool normalized, uint32_t stride_in_bytes, GLvoid* offset) {
  // replaces & frees shared pointer if present; copies the shared pointer of buffer => buffer is not deleted.
  vertexBuffers_[idx] = buffer.ptr_;

  //  assert(buffer.target() == BufferTarget::ARRAY_BUFFER); What about element indexes?

  GLuint oldvao = bindTransparently();

  buffer.bind();

  if (type == AttributeType::INT || type == AttributeType::UNSIGNED_INT) {
    glVertexAttribIPointer(static_cast<GLuint>(idx), static_cast<GLint>(numComponents), static_cast<GLenum>(type),
                           static_cast<GLuint>(stride_in_bytes), offset);
  } else {
    glVertexAttribPointer(static_cast<GLuint>(idx), static_cast<GLint>(numComponents), static_cast<GLenum>(type),
                          static_cast<GLboolean>(normalized), static_cast<GLuint>(stride_in_bytes), offset);
  }

  glEnableVertexAttribArray(static_cast<GLuint>(idx));

  releaseTransparently(oldvao);

  buffer.release();
  CheckGlError();
}

} /* namespace rv */

#endif /* INCLUDE_RV_GLVERTEXARRAY_H_ */

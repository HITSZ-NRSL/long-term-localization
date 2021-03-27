#ifndef INCLUDE_RV_GLBUFFER_H_
#define INCLUDE_RV_GLBUFFER_H_

#include <cassert>
#include <memory>
#include <string>
#include <vector>

#include "GlObject.h"

namespace glow {

class GlVertexArray;
class TransformFeedback;

/** \brief target to which the buffer is bound
 *
 * ARRAY_BUFFER Vertex attributes
 * ELEMENT_ARRAY_BUFFER Vertex array indices
 * TRANSFORM_FEEDBACK_BUFFER
 * TEXTURE_BUFFER
 */
enum class BufferTarget {
  ARRAY_BUFFER = GL_ARRAY_BUFFER,
  ELEMENT_ARRAY_BUFFER = GL_ELEMENT_ARRAY_BUFFER,
  TRANSFORM_FEEDBACK_BUFFER = GL_TRANSFORM_FEEDBACK_BUFFER,
  TEXTURE_BUFFER = GL_TEXTURE_BUFFER
  // TODO: other buffer types.
  // GL_ATOMIC_COUNTER_BUFFER, GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, GL_DRAW_INDIRECT_BUFFER,
  // GL_DISPATCH_INDIRECT_BUFFER,
  // GL_PIXEL_PACK_BUFFER, GL_PIXEL_UNPACK_BUFFER, GL_QUERY_BUFFER, GL_SHADER_STORAGE_BUFFER, GL_TEXTURE_BUFFER,
  // GL_TRANSFORM_FEEDBACK_BUFFER, or GL_UNIFORM_BUFFER
};

/** \brief Main usage of the buffer influencing the memory, where the buffer is stored.
 *
 * STREAM_DRAW The data store contents will be specified once by the application,
 *    and sourced at most a few times.
 * STREAM_READ The data store contents will be specified once by reading data from
 *     the GL, and queried at most a few times by the application.
 * STREAM_COPY The data store contents will be specified once by reading data from
 *    the GL, and sourced at most a few times
 * STATIC_DRAW The data store contents will be specified once by the application,
 *    and sourced many times.
 * STATIC_READ The data store contents will be specified once by reading data from
 *    the GL, and queried many times by the application.
 * STATIC_COPY The data store contents will be specified once by reading data from
 *    the GL, and sourced many times.
 * DYNAMIC_DRAW The data store contents will be respecified repeatedly by the ap-
 *    plication, and sourced many times.
 * DYNAMIC_READ The data store contents will be respecified repeatedly by reading
 *    data from the GL, and queried many times by the application.
 * DYNAMIC_COPY The data store contents will be respecified repeatedly by reading
 *    data from the GL, and sourced many times.
 *
 * Short:
 *  DRAW: user writes data, but not will not read.
 *  READ: user will not write, but read data.
 *  COPY: user will not write nor read it.
 *
 *  STATIC: changed once.
 *  DYNAMIC: changed occasionally.
 *  STREAM: change every time used, or almost every time.
 */
enum class BufferUsage {
  STREAM_DRAW = GL_STREAM_DRAW,
  STREAM_READ = GL_STREAM_READ,
  STREAM_COPY = GL_STREAM_COPY,
  STATIC_DRAW = GL_STATIC_DRAW,
  STATIC_READ = GL_STATIC_READ,
  STATIC_COPY = GL_STATIC_COPY,
  DYNAMIC_DRAW = GL_DYNAMIC_DRAW,
  DYNAMIC_READ = GL_DYNAMIC_READ,
  DYNAMIC_COPY = GL_DYNAMIC_COPY
};

/**
 * \brief object for OpenGL's buffer object.
 *
 * The buffer internally only reallocates memory if the assigned data exceeds the capacity.
 * Therefore, you can specify a buffer capacity, when constructing the buffer or by using
 * the reserve method.
 *
 *
 * OpenGL 4.5 Specification, p. 28:
 * > Buffer objects contain a data store holding a fixed-sized allocation of server
 * > memory, and provide a mechanism to allocate, initialize, read from, and write to
 * > such memory.  Under certain circumstances, the data store of a buffer object may
 * > be shared between the client and server and accessed simultaneously by both.
 *
 *
 * Note:
 *   Somewhere, I read that using glBufferData (allocation + copy) might be faster than
 *   calling glBufferSubData to replace part of the data. Another option was the usage of
 *   double buffered Buffers.
 *
 *   https://www.opengl.org/wiki/Vertex_Specification_Best_Practices
 *
 *   -- What about memory alignment requirements?
 *   -- Can we somehow simplify the setAttributeData of the VertexArray with information of the Buffer object.
 *
 * Fixme: Separate classes for each buffer binding target?
 *
 *    Something like GlBuffer <|--- GlArrayBuffer, GlElementArrayBuffer, GlTransformFeedbackBuffer, GlTextureBuffer, ...
 *
 * \author behley
 */
template <class T>
class GlBuffer : public GlObject {
 public:
  friend class GlVertexArray;
  friend class GlTransformFeedback;
  friend class GlTextureBuffer;

  /** \brief Initialize buffer with target = ARRAY_BUFFER using usage = STATIC_DRAW
   *
   *  \see setTarget, setUsage.
   **/
  //    GlBuffer();
  /** \brief Initialize specific buffer for given target and usage. **/
  GlBuffer(BufferTarget target, BufferUsage usage);

  /** \brief the capacity of the buffer. **/
  size_t capacity() const;
  /** \brief number of elements in the buffer. **/
  size_t size() const;

  /** \brief assign given (host) data to the buffer.
   *
   *  Replaces the buffer content with the given contents and resizes the
   *
   *
   *  Causes a reallocation if the type of the buffer changed or the capacity
   *  is not sufficient to hold the new data..
   **/
  template <class A>
  void assign(const std::vector<T, A>& data);
  void assign(const T* data, const uint32_t n);
  void assign(const GlBuffer<T>& other);
  /** TODO: \brief assign the data with iterators. **/
  //    template<class ConstIterator>
  //    void assign(const ConstIterator& begin, const ConstIterator& end);
  /** \brief replace buffer's data with given (host) data.
   *
   *  The method does only replace data, but does not increase or
   *  decrease the capacity of the buffer. Furthermore, it does not
   *  increase or decrease the size. Only data inside [0, size-1] is
   *  replaced!
   *
   *
   */
  template <class A>
  void replace(uint32_t offset, const std::vector<T, A>& data);
  void replace(uint32_t offset, const T* data, const uint32_t n);

  void insert(uint32_t offset, const T& value);

  // FIXME: have common naming for getting the data.

  /** \brief get data from the buffer. **/
  void get(std::vector<T>& data);

  /** \brief get data in range [start, start + size] from the buffer. **/
  void get(std::vector<T>& data, uint32_t start, uint32_t size);

  /** \brief reserve num_elements of T in memory. **/
  void reserve(uint32_t num_elements);

  /** \brief set the buffer's target.
   *
   *  Changes to the target of the buffer will be effective with the next assign.
   **/
  //    void setTarget(BufferTarget target);
  /** \brief set the current usage of the buffer.
   *
   *  Changes to the usage of the buffer will be effective with the next assign.
   **/
  //    void setUsage(BufferUsage usage);
  /** \brief resizes the buffer to the given new_size.
   *
   * If the capacity of the buffer is not large enough, a reallocation and copy
   * of the old data up to min(new_size, old_size) happens.
   *
   * Beware: data beyond already assigned data will be uninitialized, i.e., if
   * old size was 10 and new size is 20 than all elements 11, 12, ..., will be
   * uninitialized.
   */
  void resize(uint32_t new_size);

  BufferTarget target() const;
  BufferUsage usage() const;

  void bind() override;
  void release() override;

  /** \brief copy all content to other buffer starting at given offset **/
  void copyTo(GlBuffer<T>& other, uint32_t other_offset = 0);

  /** \brief copy content from [offset, offset+size] into other buffer [other_offset, other_offset+size] **/
  void copyTo(uint32_t offset, uint32_t size, GlBuffer<T>& other, uint32_t other_offset);

  /** \brief get memory usage of buffer in bytes. **/
  uint32_t memorySize() const { return capacity_ * dataSize_; }

 protected:
  /** \brief bind vertex array object only if needed and return overwritten vertex array object. **/
  GLuint bindTransparently();
  /** \brief release vertex array object and restore state before calling bindTranparently. **/
  void releaseTransparently(GLuint old_buffer);

  static GLuint boundBufferObject_;

  GLenum target_;
  size_t dataSize_{sizeof(T)};
  GLenum usage_;

  uint32_t capacity_{0};
  uint32_t size_{0};
};

template <class T>
GLuint GlBuffer<T>::boundBufferObject_ = 0;

template <class T>
GlBuffer<T>::GlBuffer(BufferTarget target, BufferUsage usage)
    : target_(static_cast<GLenum>(target)), usage_(static_cast<GLenum>(usage)) {
  glGenBuffers(1, &id_);
  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteBuffers(1, ptr);
    delete ptr;
  });
}

template <class T>
void GlBuffer<T>::bind() {
  boundBufferObject_ = id_;
  glBindBuffer(target_, id_);
  // FIXME: remove explicit checks?!
  //  CheckGlError();
}

template <class T>
void GlBuffer<T>::release() {
  boundBufferObject_ = 0;
  glBindBuffer(target_, 0);
  //  CheckGlError();
}

template <class T>
BufferTarget GlBuffer<T>::target() const {
  // needed?
  return static_cast<BufferTarget>(target_);
}

template <class T>
BufferUsage GlBuffer<T>::usage() const {
  // needed?
  return static_cast<BufferUsage>(usage_);
}

template <class T>
template <class A>
void GlBuffer<T>::assign(const std::vector<T, A>& data) {
  // the allocator is essentially irrelevant, but it allows to even use PCL's vectors or Eigen's vectors...
  assign(&data[0], data.size());
}

template <class T>
void GlBuffer<T>::assign(const T* data, const uint32_t n) {
  GLuint old_buffer = bindTransparently();

  if (capacity_ < n)  // reallocation needed?
  {
    glBufferData(target_, dataSize_ * n, 0, usage_);
    capacity_ = n;
  }

  glBufferSubData(target_, 0, dataSize_ * n, data);
  size_ = n;

  releaseTransparently(old_buffer);

  CheckGlError();
}

template <class T>
template <class A>
void GlBuffer<T>::replace(uint32_t offset, const std::vector<T, A>& data) {
  replace(offset, &data[0], data.size());
}

template <class T>
void GlBuffer<T>::replace(uint32_t offset, const T* data, const uint32_t n) {
  if (offset >= size_) return;  // nothing to replace.

  GLuint old_buffer = bindTransparently();

  glBufferSubData(target_, offset * dataSize_, dataSize_ * std::min(n, size_ - offset), data);

  releaseTransparently(old_buffer);

  CheckGlError();
}

template <class T>
void GlBuffer<T>::insert(uint32_t offset, const T& value) {
  replace(offset, &value, 1);
}

template <class T>
void GlBuffer<T>::get(std::vector<T>& data) {
  get(data, 0, size_);
}

template <class T>
void GlBuffer<T>::get(std::vector<T>& data, uint32_t start, uint32_t size) {
  size = std::min(size, size_ - start);  // ensure valid output size

  data.clear();
  data.resize(size);

  GLuint old_buffer = bindTransparently();
  glGetBufferSubData(target_, start * dataSize_, size * dataSize_, &data[0]);
  releaseTransparently(old_buffer);
}

template <class T>
void GlBuffer<T>::reserve(uint32_t num_elements) {
  if (capacity_ >= num_elements) return;  // already enough space.

  GLuint old_buffer = bindTransparently();

  std::vector<T> data(size_);
  if (size_ > 0) {
    // FIXME: have copy buffer here, not download & upload again.
    glGetBufferSubData(target_, 0, dataSize_ * size_, &data[0]);
  }

  // resize buffer.
  glBufferData(target_, dataSize_ * num_elements, nullptr, static_cast<GLenum>(usage_));

  if (size_ > 0) {
    glBufferSubData(target_, 0, dataSize_ * size_, &data[0]);
  }

  capacity_ = num_elements;
  releaseTransparently(old_buffer);

  CheckGlError();
}

/** \brief assign given (device) data to the buffer. **/
template <class T>
void GlBuffer<T>::assign(const GlBuffer<T>& other) {
  reserve(other.size());

  glBindBuffer(GL_COPY_READ_BUFFER, other.id_);
  glBindBuffer(GL_COPY_WRITE_BUFFER, id_);

  glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, dataSize_ * other.size_);

  glBindBuffer(GL_COPY_READ_BUFFER, 0);
  glBindBuffer(GL_COPY_WRITE_BUFFER, 0);

  size_ = other.size_;
}

template <class T>
size_t GlBuffer<T>::size() const {
  return size_;
}

template <class T>
size_t GlBuffer<T>::capacity() const {
  return capacity_;
}

template <class T>
void GlBuffer<T>::resize(uint32_t new_size) {
  reserve(new_size);
  size_ = new_size;
}

template <class T>
GLuint GlBuffer<T>::bindTransparently() {
  if (boundBufferObject_ == id_) return id_;

  // FIXME: boundBufferObject currently does not distinguish between different buffer targets:
  // FIXME: Thus, it would be easier to have different subclasses with corresponding types
  //        or template arguments GlBuffer<int BufferType, typename T>?

  glBindBuffer(target_, id_);

  return boundBufferObject_;
}

template <class T>
void GlBuffer<T>::releaseTransparently(GLuint old_buffer) {
  if (old_buffer == id_) return;  // nothing changed.

  glBindBuffer(target_, old_buffer);
}

template <class T>
void GlBuffer<T>::copyTo(GlBuffer<T>& other, uint32_t other_offset) {
  copyTo(0, size_, other, other_offset);
}

template <class T>
void GlBuffer<T>::copyTo(uint32_t offset, uint32_t size, GlBuffer<T>& other, uint32_t other_offset) {
  glBindBuffer(GL_COPY_READ_BUFFER, id_);
  glBindBuffer(GL_COPY_WRITE_BUFFER, other.id());

  glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, offset * dataSize_, other_offset * dataSize_,
                      size * dataSize_);

  glBindBuffer(GL_COPY_READ_BUFFER, 0);
  glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

} /* namespace rv */

#endif /* INCLUDE_RV_GLBUFFER_H_ */

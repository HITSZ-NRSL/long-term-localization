#ifndef SRC_OPENGL_GLOBJECT_H_
#define SRC_OPENGL_GLOBJECT_H_

#include "glbase.h"
#include <memory>

namespace glow {

/** \brief Base class for OpenGL objects.
 *
 *  Each object in OpenGL has an object identifier used to refer to an object in
 *  the device memory. Therefore, each GlObject has also an identifier. To simplify,
 *  the resource management of an GlObject, each object should use the shared pointer
 *  to delete the OpenGL resource on the device if no reference to the shared pointer
 *  exists anymore. This also ensures that we can take ownership of an GlObject by
 *  copying the shared pointer. As long as the one instance of that shared pointer
 *  exists, we are guaranteed to have a valid OpenGL object with the given identifier.
 *
 *  If the reference count of the shared pointer is zero, the custom deleter specified
 *  by each implementation of GlObject frees the device memory.
 *
 *  \author behley
 */
class GlObject {
 public:
  virtual ~GlObject() {}

  /** \brief return the OpenGL's object identifier. **/
  inline virtual GLuint id() const { return id_; }

  /** \brief bind (or activate) the object. **/
  virtual void bind() = 0;

  /** \brief unbind (or deactive) the object. **/
  virtual void release() = 0;

 protected:
  GLuint id_{0};
  std::shared_ptr<GLuint> ptr_;
};

} /* namespace rv */

#endif /* SRC_OPENGL_GLOBJECT_H_ */

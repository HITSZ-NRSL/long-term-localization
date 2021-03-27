#ifndef INCLUDE_RV_SCOPEDBINDER_H_
#define INCLUDE_RV_SCOPEDBINDER_H_

#include "GlObject.h"

namespace glow {

/** \brief Automatic bind and release of an GlObject for the current scope.
 *
 *  For simple binding and releasing of GlObjects. The binder is actually templated
 *  to ensure that we can make an appropriate copy without a polymorphic clone method.
 *  A "deep" copy of the object ensures that the GlObject's lifetime is longer then
 *  the lifetime of the ScopedBinder.
 *
 *  \author behley
 */
template <class GlObject>
class ScopedBinder {
 public:
  ScopedBinder(const GlObject& object);
  ~ScopedBinder();

 protected:
  GlObject object_;  // active copy of the object ensures that the lifetime of the object is as long as the binder.
};

template <class GlObject>
ScopedBinder<GlObject>::ScopedBinder(const GlObject& object)
    : object_(object) {
  object_.bind();
}

template <class GlObject>
ScopedBinder<GlObject>::~ScopedBinder() {
  object_.release();
}

} /* namespace rv */

#endif /* INCLUDE_RV_SCOPEDBINDER_H_ */

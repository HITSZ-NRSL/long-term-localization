#ifndef SRC_UTIL_X11OFFSCREENCONTEXT_H_
#define SRC_UTIL_X11OFFSCREENCONTEXT_H_

#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/glx.h>

namespace glow {

/** \brief simple offscreen context under X11.
 *
 *  On creation a offscreen context is generated, which binds a Pbuffer.
 *
 *  \author behley
 */
class X11OffscreenContext {
 public:
  X11OffscreenContext(int32_t major_version = 4, int32_t minor_version = 5);
  ~X11OffscreenContext();

  //  void makeCurrent();

 protected:
  Display* dpy{nullptr};
  GLXContext ctx{nullptr};
  GLXPbuffer pbuf;
};
}
#endif /* SRC_UTIL_X11OFFSCREENCONTEXT_H_ */

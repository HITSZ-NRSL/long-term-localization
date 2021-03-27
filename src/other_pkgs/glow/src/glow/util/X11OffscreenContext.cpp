#include "X11OffscreenContext.h"
#include <iostream>
#include <stdexcept>

namespace glow {

typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);
typedef Bool (*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);

X11OffscreenContext::X11OffscreenContext(int32_t major_version, int32_t minor_version) {
  static int visual_attribs[] = {None};
  int context_attribs[] = {GLX_CONTEXT_MAJOR_VERSION_ARB,
                           major_version,
                           GLX_CONTEXT_MINOR_VERSION_ARB,
                           minor_version,
                           GLX_CONTEXT_PROFILE_MASK_ARB,
                           GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
                           None};
  int fbcount = 0;
  GLXFBConfig* fbc = NULL;
  glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
  glXMakeContextCurrentARBProc glXMakeContextCurrentARB = 0;

  if (!(dpy = XOpenDisplay(0))) throw std::runtime_error("Failed to open display.");

  /* get framebuffer configs, any is usable (might want to add proper attribs) */
  if (!(fbc = glXChooseFBConfig(dpy, DefaultScreen(dpy), visual_attribs, &fbcount)))
    throw std::runtime_error("Failed to get FBConfig.");

  if (fbcount == 0) throw std::runtime_error("Failed to get FBConfig.");

  /* get the required extensions */
  glXCreateContextAttribsARB =
      (glXCreateContextAttribsARBProc)glXGetProcAddressARB((const GLubyte*)"glXCreateContextAttribsARB");
  glXMakeContextCurrentARB =
      (glXMakeContextCurrentARBProc)glXGetProcAddressARB((const GLubyte*)"glXMakeContextCurrent");
  if (!(glXCreateContextAttribsARB && glXMakeContextCurrentARB))
    throw std::runtime_error("missing support for GLX_ARB_create_context.");

  /* create a context using glXCreateContextAttribsARB */
  if (!(ctx = glXCreateContextAttribsARB(dpy, fbc[0], 0, True, context_attribs)))
    throw std::runtime_error("Failed to create opengl context.");

  /* create temporary pbuffer */
  int pbuffer_attribs[] = {GLX_PBUFFER_WIDTH, 640, GLX_PBUFFER_HEIGHT, 480, None};
  pbuf = glXCreatePbuffer(dpy, fbc[0], pbuffer_attribs);

  XFree(fbc);
  XSync(dpy, False);

  /* try to make it the current context */
  if (!glXMakeContextCurrent(dpy, pbuf, pbuf, ctx)) {
    /* some drivers does not support context without default framebuffer, so fallback on
     * using the default window.
     */
    std::cout << "not using pbuffer." << std::endl;
    if (!glXMakeContextCurrent(dpy, DefaultRootWindow(dpy), DefaultRootWindow(dpy), ctx))
      throw std::runtime_error("Failed to make current.");
  }
}

X11OffscreenContext::~X11OffscreenContext() {
  if (ctx != nullptr) glXDestroyContext(dpy, ctx);
  if (dpy != nullptr) XCloseDisplay(dpy);
}
}

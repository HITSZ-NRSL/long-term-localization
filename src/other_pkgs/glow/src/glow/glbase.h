#ifndef RV_GLPLATFORM_H
#define RV_GLPLATFORM_H

/**
 * some generic definitions needed by most OpenGL related functions.
 */

#include <iostream>
#include <GL/glew.h>

#include "glexception.h"
#include <cassert>

// If no version is specified, fallback to OpenGL version 3.30
#ifndef __GL_VERSION
#define __GL_VERSION 330L
#endif

namespace glow {

inline void _CheckGlError(const char *sFile, const int nLine) {
  GLenum glError = glGetError();
  if (glError != GL_NO_ERROR) {
    std::cerr << "OpenGL Error: " << gluErrorString(glError) << "(" << glError << ")" << std::endl;
    std::cerr << "In: " << sFile << " on Line: " << nLine << std::endl;
    throw std::runtime_error("OpenGL error detected.");
  };
}

static bool glewInitiatalized = false;

// inspired by CheckGlDieOnError in pangolin.
#undef CheckGlError

//#define CheckGlError() (glow::_CheckGlError(__FILE__, __LINE__))

#ifdef NDEBUG
#define CheckGlError() (static_cast<void>(0))
#else
#define CheckGlError() (glow::_CheckGlError(__FILE__, __LINE__))
#endif

inline void inititializeGLEW() {
  if (!glewInitiatalized) {
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
      std::cerr << glewGetErrorString(err) << std::endl;
      throw std::runtime_error("Failed to initialize GLEW.");
    }

    std::cout << "GLEW initialized." << std::endl;
    glewInitiatalized = true;

    int32_t version[2];
    glGetIntegerv(GL_MAJOR_VERSION, &version[0]);
    glGetIntegerv(GL_MINOR_VERSION, &version[1]);
    std::cout << "OpenGL context version: " << version[0] << "." << version[1] << std::endl;
    std::cout << "OpenGL vendor string  : " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "OpenGL renderer string: " << glGetString(GL_RENDERER) << std::endl;

    // consume here any OpenGL error and reset to NO_GL_ERROR:
    glGetError();
  }
}

#define PRINT_VALUE(CMD) std::cout << #CMD << " = " << CMD << std::endl;
#define PRINT_CLASSVALUE(CLASS, CMD) std::cout << #CLASS << "::" << #CMD << " = " << CMD << std::endl;
}

#endif

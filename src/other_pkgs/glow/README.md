# OpenGL Object Wrapper (GLOW) -- a high-level layer for OpenGL.

With this side project, I want to provide a simple high-level interface to OpenGL's objects with the following goals in mind:

1. simple resource management with automatic generation and deletion of OpenGL objects,
2. object related functions should be related to the object without the need to bind the right object at the right time,
3. catch potential errors or misuse of OpenGL functionality as early as possible,
4. allow only access to **core profile** functionality, since I needed it at some point in time.

These goals should align with the main purpose of this framework: simple, effortless usage of OpenGL's functionality. 
However, it does not aim at providing some framework for GUI creation, etc. 
A OpenGL context has to be created independently by virtue of a context/window creation framework, like:
- GLFW3 (http://www.glfw.org/)
- Qt (http://doc.qt.io/qt-5/qt5-intro.html)
- Pangolin (https://github.com/stevenlovegrove/Pangolin)

I only included a simple headless context for computations & testing.

This project is work in progress and will be extended over time. 
Most of the functionality included was implemented as it was needed by my own projects. 
Therefore, you cannot (or better should not) expect feature completness or stability of the interface.

## Related work

This project is inspired by some other frameworks, which are perfectly fine, but lack some features or functionality that I want to have in my work. 
However, this framework is heavily inspired by these frameworks:

 - Pangolin (https://github.com/stevenlovegrove/Pangolin): Cross-platform GUI framework with OpenGL wrapper objects. 
   Does not enforce the usage of core functionality. Furthermore, in some parts the code might not run on "pure" core profile contexts and 
   due to the usage of NVIDIA specific extensions with context from other GPU vendors, like ATI and Intel. 
 - OOGL (https://github.com/Overv/OOGL): didn't look deeply into this.
 - globjects (https://github.com/cginternals/globjects): Pretty complete high-level API for OpenGL. Also wrapping all objects into complete C++-classes.

## Requirements
 - GLEW
 - Eigen 3

## Features
 - *Shader compilation*: I usally have my shader files in separate text files and it's somehow annoying to move these stuff around and make it available relative to the executable. For this purpose, I build a solution based on CMAKE, which simply generates a statically initialized "storage" of shader strings, which can accessed by a call to a static method of `GlShaderCache`. It's somehow nice as it only needs cmake to generate the source, which can be then included in the build and therefore compiled into a library or executable.

 
## Future work
 - More documentation, examples, etc.
 - Enable/disable functionality for given OpenGL version by virtue of compile flags.
 - Feature completness for OpenGL version 4.5 (core profile)
 - Cross-plattform functionality
 - Extend GLSL parser with additional convenience functions a la Pangolin.


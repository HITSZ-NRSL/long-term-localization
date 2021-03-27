#ifndef GLOW_GLSHADER_H_
#define GLOW_GLSHADER_H_

#include "GlObject.h"
#include <string>
#include <memory>
#include <vector>

namespace glow {

enum class ShaderType {
  VERTEX_SHADER = GL_VERTEX_SHADER,
  TESS_CONTROL_SHADER = GL_TESS_CONTROL_SHADER,
  TESS_EVALUTION_SHADER = GL_TESS_EVALUATION_SHADER,
  GEOMETRY_SHADER = GL_GEOMETRY_SHADER,
  FRAGMENT_SHADER = GL_FRAGMENT_SHADER,
  COMPUTE_SHADER = GL_COMPUTE_SHADER
};

/**
 * \brief Object for OpenGL's shader
 *
 * The \a GlShader offer methods initialize and handle OpenGL shaders.
 * On initialization via constructor or the method fromFile(), the OpenGL
 * shader object is created and the source is pre-processed. Finally, the
 * pre-processed code is compiled. The GlShader can be then attached to a
 * GlProgram object.
 *
 * Inspired by Pangolin, we also add #include preprocessing directive to
 * allow inclusion of common code.
 *
 * \author behley
 */
class GlShader : public GlObject {
 public:
  friend class GlProgram;

  /** \brief Create Shader of a specific \p type with given \p source.
   *
   *  Creates the shader object with \a type and compiles the given \a source.
   *
   *  \throw GlShaderError
   **/
  GlShader(const ShaderType& type, const std::string& source, bool useCache = false);

  /** \brief return the type of the shader **/
  ShaderType type() const;

  /** \brief Create Shader from given file with \p filename
   *
   *  \return compiled shader
   *  \throw GlShaderError if reading of file or shader compilation fails.
   **/
  static GlShader fromFile(const ShaderType& type, const std::string& filename);

  /** \brief Create Shader from compiled cache source with given \p filename
   *
   *  \return compiled shader
   *  \throw GlShaderError if cache does not contain given file or shader compilation fails.
   */
  static GlShader fromCache(const ShaderType& type, const std::string& filename);

 protected:
  // hide bind() and release()
  void bind() override;
  void release() override;

  ShaderType type_;

  static std::string getCachedSource(const std::string& filename);
  static std::string readSource(const std::string& filename);

  std::string preprocess(const std::string& source);

  struct Attribute {
   public:
    std::string type;
    std::string name;
  };

  // shader meta information accessible from GlProgram.
  std::string filename;
  bool useCache_{false};

  std::vector<Attribute> inAttribs_;
  std::vector<Attribute> outAttribs_;
};
}

#endif /* INCLUDE_GLOW_GLSHADER_H_ */

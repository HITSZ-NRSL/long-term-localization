#ifndef GLOW_GLSHADERCACHE_H_
#define GLOW_GLSHADERCACHE_H_

#include <map>
#include <memory>
#include <string>

namespace glow {

/** \brief Caching of shader source files.
 *
 *  The cache is a singleton and therefore can be only accessed via the getInstance() method.
 *  Currently only shaders with distinct basenames are supported, since same shader names in
 *  different  directories are not considered.
 *
 *  \author behley
 */
class GlShaderCache {
 public:

  /** \brief get cache instance. **/
  static GlShaderCache& getInstance();

  /** \brief has source cached for given filename. **/
  bool hasSource(const std::string& filename);

  /** \brief get cached source. **/
  std::string getSource(const std::string& filename);

  /** \brief add source for given cache key value. **/
  void insertSource(const std::string& filename, const std::string& source);
  /** \brief read and add source from a file **/
  void insertSource(const std::string& filename);

 protected:
  GlShaderCache();
  GlShaderCache(const GlShaderCache&);
  GlShaderCache& operator=(const GlShaderCache&);

  std::string basename(const std::string& filename) const;


  std::map<std::string, std::string> sources_;
};

} /* namespace glow */

#endif /* GLOW_GLSHADERCACHE_H_ */

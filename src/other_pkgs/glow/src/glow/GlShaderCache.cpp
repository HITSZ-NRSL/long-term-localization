#include "GlShaderCache.h"

#include <iostream>
#include <fstream>

namespace glow {

GlShaderCache& GlShaderCache::getInstance() {
  static std::unique_ptr<GlShaderCache> instance(new GlShaderCache());

  return *instance;
}

bool GlShaderCache::hasSource(const std::string& filename) {
  std::string b = basename(filename);

  return (sources_.find(b) != sources_.end());
}

std::string GlShaderCache::getSource(const std::string& filename) {
  std::string b = basename(filename);

  return sources_[b];
}

void GlShaderCache::insertSource(const std::string& filename) {
  std::ios::openmode mode = std::ios::in | std::ios::ate;
  std::ifstream ifs(filename, mode);
  std::string source;
  if (ifs) {
    const std::streamsize size = static_cast<const std::streamsize>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);
    source.resize(size);
    ifs.read(const_cast<char*>(source.data()), size);
    source.resize(ifs.gcount());
    ifs.close();
  } else {
    std::cerr << "Error: Reading from file '" << filename << "' failed." << std::endl;
    source = "";
  }
  insertSource(filename, source);
}

void GlShaderCache::insertSource(const std::string& filename, const std::string& source) {
  std::string b = basename(filename);
  if (sources_.find(b) != sources_.end()) {
    std::cerr << "Warning: Overwriting cached shader source for filename '" << filename << "'" << std::endl;
  }
  sources_[b] = source;
}

std::string GlShaderCache::basename(const std::string& filename) const {
  std::string::size_type n = filename.rfind("/");
  if (n == std::string::npos) return filename;

  return filename.substr(n);
}

GlShaderCache::GlShaderCache() {}
GlShaderCache::GlShaderCache(const GlShaderCache&){};

} /* namespace glow */

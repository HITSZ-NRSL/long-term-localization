#include "GlShader.h"

#include <boost/tokenizer.hpp>
#include <fstream>
#include <sstream>
#include <vector>

#include "GlShaderCache.h"

namespace glow {

GlShader::GlShader(const ShaderType& type, const std::string& source, bool useCache) {
  id_ = glCreateShader(static_cast<GLenum>(type));
  type_ = type;
  useCache_ = useCache;

  std::string preprocessed = preprocess(source);

  const char* src = preprocessed.c_str();
  glShaderSource(id_, 1, &src, nullptr);

  glCompileShader(id_);

  GLint success = 1337;
  glGetShaderiv(id_, GL_COMPILE_STATUS, &success);
  if (success == GL_FALSE) {
    GLint log_size = 0;
    glGetShaderiv(id_, GL_INFO_LOG_LENGTH, &log_size);

    std::vector<GLchar> error_string(log_size);
    glGetShaderInfoLog(id_, log_size, &log_size, &error_string[0]);

    throw GlShaderError(std::string(&error_string[0], log_size));
  }

  ptr_ = std::shared_ptr<GLuint>(new GLuint(id_), [](GLuint* ptr) {
    glDeleteShader(*ptr);
    delete ptr;
  });
}

ShaderType GlShader::type() const {
  return type_;
}

GlShader GlShader::fromFile(const ShaderType& type, const std::string& filename) {
  std::string source = readSource(filename);

  try {
    // exploit move semantics to generate an rvalue.
    GlShader shader(type, source);
    shader.filename = filename;
    return shader;
  } catch (const GlShaderError& err) {
    // rethrow error with filename attached.
    std::string error_msg = filename;
    error_msg += ": " + std::string(err.what());
    throw GlShaderError(error_msg);
  }
}

GlShader GlShader::fromCache(const ShaderType& type, const std::string& filename) {
  std::string source = getCachedSource(filename);

  try {
    // exploit move semantics to generate an rvalue.
    GlShader shader(type, source, true);
    shader.filename = filename;
    return shader;
  } catch (const GlShaderError& err) {
    // rethrow error with filename attached.
    std::string error_msg = filename;
    error_msg += ": " + std::string(err.what());
    throw GlShaderError(error_msg);
  }
}

void GlShader::bind() {}

void GlShader::release() {}

std::string trim(const std::string& str, const std::string& whitespaces = " \0\t\n\r\x0B") {
  int32_t beg = 0;
  int32_t end = 0;

  /** find the beginning **/
  for (beg = 0; beg < (int32_t)str.size(); ++beg) {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i) {
      if (str[beg] == whitespaces[i]) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  /** find the end **/
  for (end = int32_t(str.size()) - 1; end > beg; --end) {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i) {
      if (str[end] == whitespaces[i]) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  return str.substr(beg, end - beg + 1);
}

std::vector<std::string> split(const std::string& line, const std::string& delim, bool skipEmpty = false) {
  std::vector<std::string> tokens;

  boost::char_separator<char> sep(delim.c_str(), "", (skipEmpty ? boost::drop_empty_tokens : boost::keep_empty_tokens));
  boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);

  for (auto it = tokenizer.begin(); it != tokenizer.end(); ++it) tokens.push_back(*it);

  return tokens;
}

std::string GlShader::readSource(const std::string& filename) {
  std::ifstream in_file(filename);
  if (!in_file.is_open()) throw GlShaderError("Unable to open shader file: " + filename);

  std::string line;
  std::string source;
  in_file.peek();
  while (!in_file.eof()) {
    std::getline(in_file, line);
    source += line + "\n";
    in_file.peek();
  }

  in_file.close();

  return source;
}

std::string GlShader::getCachedSource(const std::string& filename) {
  GlShaderCache& cache = GlShaderCache::getInstance();

  if (!cache.hasSource(filename)) throw GlShaderError("Cache does not contain entry with name '" + filename + "'");

  return cache.getSource(filename);
}

std::string GlShader::preprocess(const std::string& source) {
  std::stringstream in(source);
  std::stringstream out;
  std::string line;

  int32_t line_no = 0;

  in.peek();
  while (!in.eof()) {
    std::getline(in, line);
    line = trim(line);

    std::vector<std::string> tokens = split(line, " ");

    if (tokens.size() == 2 && tokens[0] == "#include") {
      if (tokens.size() != 2) {
        throw GlShaderError("Filename of included file missing in line " + std::to_string(line_no));
      }

      std::string filename = trim(tokens[1], "\"");
      std::string include_source;
      if (useCache_)
        include_source = getCachedSource(filename);
      else
        include_source = readSource(filename);

      out << preprocess(include_source) << std::endl;
    } else if (tokens.size() > 3 && tokens[0] == "in") {
      Attribute attr;
      attr.name = trim(tokens[2], ";");
      attr.type = tokens[1];

      inAttribs_.push_back(attr);

      out << line << std::endl;
    } else if (tokens.size() > 4 && tokens[0] == "flat" && tokens[1] == "in") {
      Attribute attr;
      attr.name = trim(tokens[3], ";");
      attr.type = tokens[2];

      inAttribs_.push_back(attr);

      out << line << std::endl;
    } else if (tokens.size() > 3 && tokens[0] == "out") {
      Attribute attr;
      attr.name = trim(tokens[2], ";");
      attr.type = tokens[1];

      outAttribs_.push_back(attr);

      out << line << std::endl;
    } else if (tokens.size() > 3 && tokens[0] == "flat" && tokens[1] == "out") {
      Attribute attr;
      attr.name = trim(tokens[3], ";");
      attr.type = tokens[2];

      outAttribs_.push_back(attr);

      out << line << std::endl;
    } else {
      out << line << std::endl;
    }

    in.peek();
  }

  if (out.str().size() == 0) std::cerr << "Warning: empty shader source." << std::endl;
  return out.str();
}
}
